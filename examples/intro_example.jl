using AlgamesDriving
using Algames
using StaticArrays
using LinearAlgebra
using MeshCat

# Generate Autonomous Driving Scenario

# Create Roadway
roadway_opts = MergingRoadwayOptions()
roadway = build_roadway(roadway_opts)

# Create Dynamics Model
p = 3
model = UnicycleGame(p=p)

# Create Players
players = Vector{Player}(undef, p)
# For each player specify the start and goal state
players[1] = Player(model, roadway.lane[1],
    x0=VehicleState(0.0, 0.1, 0.0, 0.3),
    xf=VehicleState(4.0, 0.1, 0.0, 0.3))
players[2] = Player(model, roadway.lane[2],
    x0=VehicleState(0.0,-0.1, 0.0, 0.3),
    xf=VehicleState(4.0,-0.1, 0.0, 0.3))
players[3] = Player(model, roadway.lane[3],
    x0=VehicleState(0.2,-0.1, 0.0, 0.3),
    xf=VehicleState(4.0, 0.1, 0.0, 0.3))

# Create Scenario
sce = Scenario(model, roadway, players)

# Create Dynamic Game Problem
N = 20 # number of time steps
dt = 0.1 # time step
solver_opts = Options() # solver options
prob = GameProblem(N, dt, sce, solver_opts) # atonomous driving problem
@time newton_solve!(prob) # solve the problem

# Plotting vehicles' trajectories and the solver's progress
using Plots
plot(model, prob.pdtraj.pr)
plot(prob.stats)

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
