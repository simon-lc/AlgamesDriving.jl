using AlgamesDriving
using Algames
using StaticArrays
using LinearAlgebra
using MeshCat

# Generate Autonomous Driving Scenario

# Create Roadway
roadway_opts = FourIntersectionRoadwayOptions()
roadway = build_roadway(roadway_opts)

@show test_centerline_continuity(roadway.lane[1].centerline, sample=250)
@show test_centerline_continuity(roadway.lane[5].centerline, sample=250)
@show test_centerline_continuity(roadway.lane[6].centerline, sample=250)


# Create Dynamics Model
p = 3
model = UnicycleGame(p=p)

players = Vector{Player}(undef, p)
# For each player specify the start and goal state
# west_straight - east
players[1] = Player(model, roadway.lane[1],
    Q=Diagonal(SVector{model.ni[1]}([1,1,0.1,1])),
    x0=VehicleState(-2.2, -0.1, 0.0, 0.9),
    xf=VehicleState( 0.5, -0.1, 0.0, 0.9))

# west_right - east
players[2] = Player(model, roadway.lane[5],
    Q=Diagonal(SVector{model.ni[2]}([1,1,0.1,1])),
    x0=VehicleState(-1.8, -0.1, 0.0, 0.9),
    xf=VehicleState(-0.1, -1.0, -π/2, 0.9))

# west_left - east
players[3] = Player(model, roadway.lane[6],
    Q=Diagonal(SVector{model.ni[3]}([1,1,0.1,1])),
    x0=VehicleState(-1.4, -0.1, 0.0, 0.9),
    xf=VehicleState( 0.1,  1.0, π/2, 0.9))

# # east - west
# players[2] = Player(model, roadway.lane[2],
#     x0=VehicleState(1.8, 0.1, 0.0, 0.3),
#     xf=VehicleState(-1.8, 0.1, 0.0, 0.3))
#
# # north - south
# players[3] = Player(model, roadway.lane[3],
#     x0=VehicleState(-0.1, 1.8, 4.71, 0.4),
#     xf=VehicleState(-0.1, -2.0, 4.71, 0.4))
#
# # # south - north
# players[4] = Player(model, roadway.lane[4],
#     x0=VehicleState(0.1, -1.6, 1.57, 0.3),
#     xf=VehicleState(0.1, 1.5, 1.57, 0.3))

# Create Scenario
sce = Scenario(model, roadway, players)

# # Create Dynamic Game Problem
N = 40 # number of time steps
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
# set_camera_birdseye!(vis, height=7)
set_env!(vis, VehicleState(0.,0.,0.,0.))
# Animate the vehicles
set_traj!(vis, model, sce, prob.pdtraj.pr)
