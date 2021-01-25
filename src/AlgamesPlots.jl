module AlgamesPlots

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
using Plots
using REPL
using Rotations
using StaticArrays
using TrajOptPlots


export
    vec_add!,
    vec_sub!,
    VecPair

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
    Circle,
    Lane
# roadway
export
    Roadway,
    RoadwayOptions,
    HighwayRoadwayOptions,
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
    clean!

include("newcode.jl")

# Struct
include("struct/vehicle_state.jl")
include("struct/starting_area.jl")
include("struct/lane.jl")
include("struct/roadway.jl")
include("struct/player.jl")

# Scenario
include("scenario/scenario.jl")

# Visualization
include("visualization/visualization_methods.jl")
include("visualization/visualize_roadway.jl")
include("visualization/visualize_player.jl")
include("visualization/visualize_scenario.jl")
include("visualization/visualize_trajectory.jl")
include("visualization/camera.jl")

# MPC
include("mpc/mpc_struct.jl")
include("mpc/mpc_methods.jl")
include("mpc/game.jl")

# Plots
include("plots/solver_plots.jl")


end # module
