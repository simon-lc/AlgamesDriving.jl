module AlgamesPlots

greet() = print("Hello World!")

#--- Added Code
using StaticArrays
using LinearAlgebra

export
    vec_add!,
    vec_sub!,
    VecPair

include("newcode.jl")
#---

end # module
