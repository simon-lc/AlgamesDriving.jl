include("mpc_helpers.jl")
T = Float64
marker = [:none, :auto, :circle, :rect, :star5,
    :diamond, :hexagon, :cross, :xcross, :utriangle,
    :dtriangle, :rtriangle, :ltriangle, :pentagon,
    :heptagon, :octagon, :star4, :star6, :star7, :star8, :vline, :hline, :x]

# Create players
p = 2
model = DoubleIntegratorGame(p=p)
model = UnicycleGame(p=p)
n = model.n
m = model.m


# Create Roadway
roadway_opts = MergingRoadwayOptions(lane_width=0.26, merging_angle=pi/12, merging_point=1.5)
roadway = build_roadway(roadway_opts)

# Create Players
players = Vector{Player{T}}(undef, p)
players[1] = Player(model, roadway.lane[5])
players[2] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)


# Define the horizon of the problem
N = 20 # N time steps
dt = 0.12 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(1*[0.0, 0.5, 1.0, 0.5]) for i=1:p] # Quadratic state cost
R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desired state
xf = [SVector{model.ni[1],T}([1.0,-0.10,0.0,0.50]),
    SVector{model.ni[2],T}([1.0,-0.10,0.0,0.75]),
    ]

# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.25*ones(p)
μ = 1.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
game_con = GameConstraintValues(N, sce)
u_max =  SVector{m,T}([1.5, 1.5, 0.75, 0.75])
u_min =  SVector{m,T}([-1.5, -1.5, -0.75, -0.75])
add_control_bound!(game_con, u_max, u_min)


# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.25, -0.30,
   -0.45, -0.08,
   pi/12,   0.0,
    0.50,   0.55,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-8
opts.α_dual = 0.0
opts.λ_max = 1.0*1e7
opts.ϵ_dyn = 1e-6
opts.ϵ_sta = 1e-6
opts.ϵ_con = 1e-6
opts.ϵ_opt = 1e-6
opts.regularize = true
# Define the game problem
prob = Algames.GameProblem(N,dt,x0,model,opts,game_obj,game_con)

# Solve the problem
newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)


# Initialize visualizers
vis = [Visualizer() for i=1:p]
open.(vis)

# Visualize trajectories
col = [:cornflowerblue, :orange]
for i = 1:p
    set_scenario!(vis[i], sce, color=col)
    set_env!(vis[i], VehicleState(1.0, 0.0, 0.0, 0.0))
    set_camera_birdseye!(vis[i], height=10.0)
end


mpc_opts = MPCOptions(M=40, h=2.5)
m_stats = MPCStatistics()
t_stats = TrajStatistics(probsize)

for i = 1:20
    xinit = random_initial_state(model, x0, amplitude=1., seed=i)
    set_state!(vis[1], model, sce, xinit)
    sleep(0.1)
end

for i = 1:1
    xinit = random_initial_state(model, x0, seed=i, amplitude=1.0)
    mpc_loop(prob, xinit, sce, vis, mpc_opts, m_stats, t_stats, seed=[3,10],
        display_traj=true, display_player=true)
end

mpc_opts = MPCOptions(M=500, h=4.0)
mpc_stats, traj_stats = run_experiment(prob, x0, sce, vis, mpc_opts,
    n_sample=100, display_traj=true, display_player=true)
x, t, e, d = process_data(model, mpc_stats, traj_stats, α=0.25, upsampling=1, atol=2.0)
get_figure(x,t,e,d)


# Plot the mismatch
mpc_opts = MPCOptions(M=400, h=2.5)
m_stats = MPCStatistics()
t_stats = TrajStatistics(probsize)

xinit = random_initial_state(model, x0, seed=1, amplitude=1.0)
mpc_loop(prob, xinit, sce, vis, mpc_opts, m_stats, t_stats, seed=[3,10],
    display_traj=true, display_player=true)

t = 5
for i = 1:2
    build_waypoint!(vis[i], players, N, color=[col[i] for j=1:p])
    set_state!(vis[i], model, sce, Algames.state(m_stats.traj[t]))
    set_line_traj!(vis[i], model, sce.player, t_stats.traj[t][i], color=[col[i] for j=1:p])
    set_waypoint_traj!(vis[i], model, sce, t_stats.traj[t][i])
    set_camera_birdseye!(vis[i], height=5.0)
end
t = 40
cumsum(t_stats.dt)[t]
for i = 1:p
    set_state!(vis[i], model, sce, Algames.state(m_stats.traj[t]))
    set_line_traj!(vis[i], model, sce.player, t_stats.traj[t][i], color=col)
    set_waypoint_traj!(vis[i], model, sce, t_stats.traj[t][i])
end




# Visualization
highway_roadway_opts = HighwayRoadwayOptions(lane_width=0.26, lane_length=1.0)
highway_vis_opts = RoadwayVisualizationOptions(bound_height=-0.001)
k = 10
for i = 1:p
    set_roadway!(vis[i], highway_roadway_opts, k=k, vis_opts=highway_vis_opts)
    settransform!(vis[i]["env/roadway$k"], Translation(-0.4, 0.0, 0.0))
end
