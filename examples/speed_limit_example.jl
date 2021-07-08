using AlgamesDriving
using Algames
using StaticArrays
using LinearAlgebra
using MeshCat


# Visualize Scenario
vis = Visualizer()
open(vis)

mutable struct DataCollectionParams
    num_agents::Int
    # One element per agent in each of the arrays below
    lanes::Array
    init_vel::Array
    target_vel::Array
    collision_params::Array
end
# Generate Autonomous Driving Scenario

# Create Roadway
roadway_opts = FourIntersectionRoadwayOptions()
roadway_opts.lane_length = 4.8
roadway_opts.lane_width = 1.2
roadway_opts.turn_radius = 0.175
roadway = build_roadway(roadway_opts)

# Create Dynamics Model
p = 2
model = UnicycleGame(p=p)
params = DataCollectionParams(p, [1,10], [0.3, 0.4], [0.6,0.8], [0.2, 0.25])

players = Vector{Player}(undef, p)
# For each player specify the start and goal state
# west_straight
players[1] = Player(model, roadway.lane[params.lanes[1]],
    Q=Diagonal(SVector{model.ni[1]}([1e-2,1,0.1,5])),
    x0=VehicleState(-2.4, -0.3, 0.0, params.init_vel[1]),
    xf=VehicleState( 2.2, -0.3, 0.0, params.target_vel[1]),
    u_min=SVector{2, Float64}([-0.36, -1.0]),
    u_max=SVector{2, Float64}([0.36, 1.0]),
    v_min=-Inf, # no constraints on the velocity
    v_max=Inf, # no constraints on the velocity
    r_col=params.collision_params[1])

# south_straight
players[2] = Player(model, roadway.lane[params.lanes[2]],
    Q=Diagonal(SVector{model.ni[2]}([1,1e-2,0.1,5])),
    x0=VehicleState(0.3, -3.7, π/2, params.init_vel[2]),
    xf=VehicleState(0.3, 2.2, π/2, params.target_vel[2]),
    u_min=SVector{2, Float64}([-0.36, -1.0]),
    u_max=SVector{2, Float64}([0.36, 1.0]),
    v_min=0.0, # constrained velocity
    v_max=1.05*params.target_vel[2], # constrained velocity
    r_col=params.collision_params[2])


# Create Scenario
sce = Scenario(model, roadway, players)

# # Create Dynamic Game Problem
N = 71 # number of time steps
dt = 0.1 # time step
solver_opts = Options() # solver options
prob = GameProblem(N, dt, sce, solver_opts) # autonomous driving problem
@time newton_solve!(prob) # solve the problem


# Build environment
player_vis_opts = PlayerVisualizationOptions(car_scale=2.5)
set_scenario!(vis, sce, player_vis_opts=player_vis_opts)
# Set camera
set_env!(vis, VehicleState(0.,0.,0.,0.))
# Animate the vehicles
set_traj!(vis, model, sce, prob.pdtraj.pr)

# Plotting vehicles' trajectories and velocities and the solver's progress
plot!(prob.stats)
plot!(model, prob.pdtraj.pr)
plot_velocity_!(model, prob.pdtraj.pr)
