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
p = 2
model = UnicycleGame(p=p)

players = Vector{Player}(undef, p)
# For each player specify the start and goal state
# west - east
players[1] = Player(model, roadway.lane[1],
    x0=VehicleState(-1.8, -0.1, 0.0, 0.3),
    xf=VehicleState(1.8, -0.1, 0.0, 0.3))

# east - west
# players[2] = Player(model, roadway.lane[2],
#     x0=VehicleState(1.8, 0.1, 0.0, 0.3),
#     xf=VehicleState(-1.8, 0.1, 0.0, 0.3))

# north - south
# players[2] = Player(model, roadway.lane[3],
#     x0=VehicleState(-0.1, 1.8, 4.71, 0.4),
#     xf=VehicleState(-0.1, -2.0, 4.71, 0.4))

# # south - north
players[2] = Player(model, roadway.lane[4],
    x0=VehicleState(0.1, -1.6, 1.57, 0.3),
    xf=VehicleState(0.1, 1.5, 1.57, 0.3))

# Create Scenario
sce = Scenario(model, roadway, players)

# # Create Dynamic Game Problem
N = 20 # number of time steps
dt = 0.1 # time step
solver_opts = Options() # solver options
prob = GameProblem(N, dt, sce, solver_opts) # atonomous driving problem
@time newton_solve!(prob) # solve the problem

# Plotting vehicles' trajectories and the solver's progress
# plot_traj_!(model, prob.pdtraj.pr)
# plot_violation_!(prob.stats)

# Visualize Scenario
vis = Visualizer()
open(vis)
# Build environment
set_scenario!(vis, sce)
# Set camera
set_camera_birdseye!(vis, height=7)
set_env!(vis, VehicleState(2.,0.,0.,0.))
# Animate the vehicles
set_traj!(vis, model, sce, prob.pdtraj.pr)
