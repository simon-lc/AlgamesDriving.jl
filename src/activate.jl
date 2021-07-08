# Run tests locally
using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
# Pkg.test("AlgamesDriving")


Pkg.activate(joinpath(@__DIR__, "../test"))
Pkg.activate(joinpath(@__DIR__, "../docs"))
