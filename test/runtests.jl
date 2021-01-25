using Test
using AlgamesPlots
using Colors
using GeometryBasics
using LinearAlgebra
using MeshCat
using Parameters
using REPL
using StaticArrays

include("vec_addsub.jl")
include("VecPair.jl")

#Struct
include("struct/vehicle_state.jl")
include("struct/starting_area.jl")
include("struct/lane.jl")
include("struct/roadway.jl")
include("struct/player.jl")

# Scenario
include("scenario/scenario.jl")
