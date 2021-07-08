module AlgamesDriving

greet() = print("Hello World!")

using Algames
using BenchmarkTools
using Colors
using CoordinateTransformations
using FileIO
using GeometryBasics
using LinearAlgebra
using MeshCat
using MeshIO
using Parameters
using REPL
using Random
using Rotations
using StaticArrays

# Struct
# vehicle_state
export
    VehicleState,
    standardize,
    specialize
# starting_area
export
    StartingArea,
    randstate
# lane
export
    CircularWall,
    Lane
# roadway
export
    Roadway,
    RoadwayOptions,
    HighwayRoadwayOptions,
    MergingRoadwayOptions,
    FourIntersectionRoadwayOptions,
    build_roadway,
    add_lane!
# player
export
    Player

# Sceanario
# scenario
export
    Scenario,
    get_state,
    get_control_bounds

# Visualization
# visualize_roadway
export
    VisualizationOptions,
    visualize_roadway!
# visualization_methods
export
    clean!,
    ModifiedMeshFileObject,
    rescale_contents,
    select_material,
    transRot
# Camera
export
    set_env!,
    set_camera!,
    set_camera_birdseye!
# visualize_player
export
    PlayerVisualizationOptions,
    WaypointVisualizationOptions,
    LineVisualizationOptions,
    set_player!,
    build_waypoint!,
    set_line_traj!
# visualize_roadway
export
    RoadwayVisualizationOptions,
    set_roadway!
# visualize_scenario
export
    set_scenario!
# visualize_trajectory
export
    set_state!,
    set_traj!,
    set_waypoint!,
    set_waypoint_traj!

# MPC
# Game
export
    get_char
# mpc_struct
export
    MPCOptions,
    MPCStatistics,
    TrajStatistics
# mpc_methods
export
    first_order_rollout,
    simulate_MPC!

# # Plots
# export


# Struct
include("struct/vehicle_state.jl")
include("struct/starting_area.jl")
include("struct/lane.jl")
include("struct/roadway.jl")
include("struct/highway_roadway.jl")
include("struct/merging_roadway.jl")
include("struct/four_intersection_roadway.jl")
include("struct/player.jl")

# Scenario
include("scenario/scenario.jl")

# Visualization
include("visualization/visualization_methods.jl")
include("visualization/camera.jl")
include("visualization/vehicle/visualize_roadway.jl")
include("visualization/vehicle/visualize_player.jl")
include("visualization/vehicle/visualize_scenario.jl")
include("visualization/vehicle/visualize_trajectory.jl")
include("visualization/drone/static_object.jl")
include("visualization/drone/constraint.jl")
include("visualization/drone/animation.jl")
include("visualization/drone/bicycle.jl")
include("visualization/drone/double_integrator.jl")
include("visualization/drone/quadrotor.jl")
include("visualization/drone/unicycle.jl")

# MPC
include("mpc/mpc_struct.jl")
include("mpc/mpc_methods.jl")
include("mpc/game.jl")

# Plots

# Utils
include("utils.jl")

end # module
