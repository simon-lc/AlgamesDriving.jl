vis = Visualizer()
open(vis)

# Create Roadway
roadway_opts = HighwayRoadwayOptions()
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 3
model = UnicycleGame(p=p)

players = Vector{Player{T}}(undef, p)
players[1] = Player(model, roadway.lane[1],
    Q=Diagonal(SVector{model.ni[1],T}(0.0, 1.0, 1.0, 1.0)),
    x0=VehicleState(0.1, 0.1, 0.0, 0.7),
    xf=VehicleState(3.6, 0.1, 0.0, 0.7))
players[2] = Player(model, roadway.lane[2],
    Q=Diagonal(SVector{model.ni[2],T}(0.0, 1.0, 1.0, 1.0)),
    x0=VehicleState(0.1,-0.1, 0.0, 0.7),
    xf=VehicleState(3.6,-0.1, 0.0, 0.7))
players[3] = Player(model, roadway.lane[3],
    Q=Diagonal(SVector{model.ni[3],T}(0.0, 1.0, 1.0, 1.0)),
    x0=VehicleState(0.3,-0.1, 0.0, 0.7),
    xf=VehicleState(3.8, 0.1, 0.0, 0.7))

# Create Scenario
sce = Scenario(model, roadway, players)

# Create AutoProblem
N = 50
dt = 0.1
solver_opts = Options()
solver_opts.inner_print = true
solver_opts.outer_print = true
prob = GameProblem(N, dt, sce, solver_opts)
@time newton_solve!(prob)

plot!(model, prob.pdtraj.pr)
plot!(prob.stats)

# Visualize trajectory
set_scenario!(vis, sce)
set_env!(vis, VehicleState(2.0, 0.0, 0.0, 0.0))
set_camera_birdseye!(vis, height=7.0)
build_waypoint!(vis, sce.player, N)

@time set_state!(vis, model, sce, Algames.state(prob.pdtraj.pr[10]))
@time set_traj!(vis, model, sce, prob.pdtraj.pr)
@time set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr)
