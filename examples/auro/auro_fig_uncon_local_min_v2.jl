# Create Roadway
roadway_opts = MergingRoadwayOptions(lane_width=0.26)
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 2
model = DoubleIntegratorGame(p=p)
n = model.n
m = model.m

# Define the horizon of the problem
N = 20 # N time steps
dt = 0.1 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(3*ones(SVector{model.ni[i],T})) for i=1:p] # Quadratic state cost
R = [Diagonal(0.3*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([1.0,+0.35,0.5,0]),
      SVector{model.ni[2],T}([1.0,-0.05,0.5,0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.2*ones(p)
μ = 100.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)

# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.2,   0.2,
   -0.45, -0.05,
    0.4,   0.5,
    0.1,   0.0,
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

players = Vector{Player{T}}(undef, p)
player_opts = HighwayPlayerOptions{model.ni[1],model.mi[1],T}()
players[1] = Player(model, roadway.lane[3])
players[2] = Player(model, roadway.lane[6])

# Create Scenario
sce = Scenario(model, roadway, players)

function get_figure(prob::Algames.GameProblem, sce::Scenario, vis_sol::Visualizer, vis_init::Visualizer)
    N = prob.probsize.N
    model = prob.model

    prob.opts.shift = 2^10
    prob.opts.α_dual = 0.0
    prob.opts.αx_dual = zeros(p)
    prob.opts.f_init = x -> (rand(x) .- 0.5)
    prob.opts.amplitude_init = 1.0

    # Set tolerances
    prob.opts.ϵ_opt = 1e-6
    prob.opts.ϵ_dyn = 1e-6
    prob.opts.ϵ_con = 1e-6
    prob.opts.ϵ_sta = 1e-6

    init = []
    sol = []
    plt_init = plot()
    plt_sol = plot()
    for k = 1:20
        # Randomize the initialization
        prob.opts.seed = k

        # Get the initial guess (random controls + dynamics rollout)
        prob.opts.outer_iter = 0
        newton_solve!(prob)
        init_traj = deepcopy(prob.pdtraj)

        # Get solution trajectory
        prob.opts.outer_iter = 20
        newton_solve!(prob)
        sol_traj = deepcopy(prob.pdtraj)

        # Only keep the solution that converged
        opt = mean(abs.(prob.stats.opt_vio[end].vio))
        sta = mean(abs.(prob.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
        con = mean(abs.(prob.stats.con_vio[end].vio))
        opts = prob.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            build_waypoint!(vis_init, sce.player, N, key=k)
            build_waypoint!(vis_sol,  sce.player, N, key=k)
            set_waypoint_traj!(vis_init, model, sce, init_traj.pr, key=k)
            set_waypoint_traj!(vis_sol,  model, sce, sol_traj.pr, key=k)

            plot!(model, init_traj.pr, plt=plt_init)
            plot!(model, sol_traj.pr, plt=plt_sol)
            push!(sol, sol_traj)
            push!(init, init_traj)
        end
    end
    set_state!(vis_init, model, sce, Algames.state(init[1].pr[1]))
    set_state!(vis_sol,  model, sce, Algames.state(sol[1].pr[1]))
    return init, sol
end

# Initialize visualizers
vis_init = Visualizer()
vis_sol = Visualizer()
open(vis_init)
open(vis_sol)

# Visualize trajectories
set_scenario!(vis_init, sce)
set_scenario!(vis_sol, sce)
set_env!(vis_init, VehicleState(1.0, 0.0, 0.0, 0.0))
set_env!(vis_sol, VehicleState(1.0, 0.0, 0.0, 0.0))
set_camera_birdseye!(vis_init, height=4.0)
set_camera_birdseye!(vis_sol, height=4.0)

init_traj, sol_traj = get_figure(prob, sce, vis_sol, vis_init)
