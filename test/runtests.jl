using Test
using AlgamesDriving
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

#Struct
include("struct/vehicle_state.jl")
include("struct/starting_area.jl")
include("struct/lane.jl")
include("struct/roadway.jl")
include("struct/player.jl")

# Scenario
include("scenario/scenario.jl")

# Visualization
include("visualization/camera.jl")

# MPC
include("mpc/mpc_struct.jl")
include("mpc/mpc_methods.jl")
