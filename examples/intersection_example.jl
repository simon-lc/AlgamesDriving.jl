using AlgamesDriving
using Algames
using StaticArrays
using LinearAlgebra
using MeshCat

# Generate Autonomous Driving Scenario

# Create Roadway
roadway_opts = FourIntersectionRoadwayOptions()
roadway = build_roadway(roadway_opts)

# Create Dynamics Model
p = 3
model = UnicycleGame(p=p)

players = Vector{Player}(undef, p)
# For each player specify the start and goal state
# west_straight
players[1] = Player(model, roadway.lane[1],
    Q=Diagonal(SVector{model.ni[1]}([1,1,0.1,1])),
    x0=VehicleState(-2.2, -0.1, 0.0, 0.9),
    xf=VehicleState( 0.5, -0.1, 0.0, 0.9))

# west_right
players[2] = Player(model, roadway.lane[2],
    Q=Diagonal(SVector{model.ni[2]}([1,1,0.1,1])),
    x0=VehicleState(-1.8, -0.1, 0.0, 0.9),
    xf=VehicleState(-0.1, -1.0, -π/2, 0.9))

# west_left
players[3] = Player(model, roadway.lane[3],
    Q=Diagonal(SVector{model.ni[3]}([1,1,0.1,1])),
    x0=VehicleState(-1.4, -0.1, 0.0, 0.9),
    xf=VehicleState( 0.1,  1.0, π/2, 0.9))

# # east_straight
# players[1] = Player(model, roadway.lane[4],
#     Q=Diagonal(SVector{model.ni[1]}([1,1,0.1,1])),
#     x0=VehicleState(2.2, 0.1, 3.14, 0.9),
#     xf=VehicleState( -0.5, 0.1, 3.14, 0.9))
#
# # east_right
# players[2] = Player(model, roadway.lane[5],
#     Q=Diagonal(SVector{model.ni[2]}([1,1,0.1,1])),
#     x0=VehicleState(1.8, 0.1, 3.14, 0.9),
#     xf=VehicleState(0.1, 1.0, π/2, 0.9))
#
# # east_left
# players[3] = Player(model, roadway.lane[6],
#     Q=Diagonal(SVector{model.ni[3]}([1,1,0.1,1])),
#     x0=VehicleState(1.4, 0.1, 3.14, 0.9),
#     xf=VehicleState( -0.1, -1.0, 3*π/2, 0.9))

# # north_straight
# players[1] = Player(model, roadway.lane[7],
#     Q=Diagonal(SVector{model.ni[1]}([1,1,0.1,1])),
#     x0=VehicleState(-0.1, 2.2, -1.57, 0.9),
#     xf=VehicleState( -0.1, -0.5, -π/2, 0.9))

# # north_right
# players[2] = Player(model, roadway.lane[8],
#     Q=Diagonal(SVector{model.ni[2]}([1,1,0.1,1])),
#     x0=VehicleState(-0.1, 1.8, -1.57, 0.9),
#     xf=VehicleState(-1.0, 0.1, -π, 0.9))

# # north_left
# players[3] = Player(model, roadway.lane[9],
#     Q=Diagonal(SVector{model.ni[3]}([1,1,0.1,1])),
#     x0=VehicleState(-0.1, 1.4, -1.57, 0.9),
#     xf=VehicleState( 1.0, -0.1, 0., 0.9))

# # south_straight
# players[1] = Player(model, roadway.lane[10],
#     Q=Diagonal(SVector{model.ni[1]}([1,1,0.1,1])),
#     x0=VehicleState(0.1, -2.2, 1.57, 0.9),
#     xf=VehicleState( 0.1, 0.5, π/2, 0.9))

# # south_right
# players[2] = Player(model, roadway.lane[11],
#     Q=Diagonal(SVector{model.ni[2]}([1,1,0.1,1])),
#     x0=VehicleState(0.1, -1.8, 1.57, 0.9),
#     xf=VehicleState(1.0, -0.1, 0., 0.9))

# # south_left
# players[3] = Player(model, roadway.lane[12],
#     Q=Diagonal(SVector{model.ni[3]}([1,1,0.1,1])),
#     x0=VehicleState(0.1, -1.4, 1.57, 0.9),
#     xf=VehicleState( -1.0, 0.1, 3.14, 0.9))

# Create Scenario
sce = Scenario(model, roadway, players)

# # Create Dynamic Game Problem
N = 40 # number of time steps
dt = 0.1 # time step
solver_opts = Options() # solver options
prob = GameProblem(N, dt, sce, solver_opts) # autonomous driving problem
@time newton_solve!(prob) # solve the problem

# Plotting vehicles' trajectories and the solver's progress
# plot!(model, prob.pdtraj.pr)
# plot!(prob.stats)

# Visualize Scenario
vis = Visualizer()
open(vis)
# Build environment
set_scenario!(vis, sce)
# Set camera
# set_camera_birdseye!(vis, height=7)
set_env!(vis, VehicleState(0.,0.,0.,0.))
# Animate the vehicles
set_traj!(vis, model, sce, prob.pdtraj.pr)
