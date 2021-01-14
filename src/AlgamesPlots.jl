module AlgamesPlots

greet() = print("Hello World!")

using Algames
using LinearAlgebra
using StaticArrays


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


include("newcode.jl")

# Struct
include("struct/vehicle_state.jl")
include("struct/starting_area.jl")
include("struct/lane.jl")

end # module
