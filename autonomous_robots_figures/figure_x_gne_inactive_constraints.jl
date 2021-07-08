include("experiment_helpers.jl")

# Create Roadway
roadway_opts = MergingRoadwayOptions(merging_point=1.4, lane_width=0.28)
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 3
model = BicycleGame(p=p)
n = model.n
m = model.m

# Define the horizon of the problem
N = 30 # N time steps
dt = 0.10 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(3*SVector{model.ni[i],T}([1., 0.8, 1., 1.])) for i=1:p] # Quadratic state cost
R = [Diagonal(0.3*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2.0, 0.08, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.08, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.08, 0.0, 0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.20*ones(p)
μ = 10.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
radius = 0.08
add_collision_avoidance!(game_con, radius)


# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.2,     0.2,    0.2,
   -0.42,   -0.10,   0.16,
    0.45,    0.60,   0.60,
    0.15,    0.0,    0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-5
opts.α_dual = 1.0
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
players[1] = Player(model, roadway.lane[3])
players[2] = Player(model, roadway.lane[3])
players[3] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)

# Initialize visualizers
vis = Visualizer()
open(vis)

# Visualize trajectories
set_scenario!(vis, sce)
set_env!(vis, VehicleState(1.3, 0.0, 0.0, 0.0))
set_camera_birdseye!(vis, height=5.0)

build_waypoint!(vis, sce.player, N, key=0)
set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr, key=0)
set_traj!(vis, model, sce, prob.pdtraj.pr)

display_ghost(vis, prob, sce)

# Verify the invertibility of the Jacobian of the KKT conditions.
residual_jacobian!(prob, prob.pdtraj)
null = nullspace(Matrix(prob.core.jac))
dim = size(null)[2]
