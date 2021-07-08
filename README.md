![CI](https://github.com/simon-lc/AlgamesDriving.jl/workflows/CI/badge.svg)
[![codecov](https://codecov.io/gh/simon-lc/AlgamesDriving.jl/branch/master/graph/badge.svg?token=UMFAFPUGBE)](https://codecov.io/gh/simon-lc/AlgamesDriving.jl)
[![](https://img.shields.io/badge/docs-dev-blue.svg)](https://simon-lc.github.io/AlgamesDriving.jl/dev)

## Purpose
A set of tools to quickly generate and visualize autonomous driving scenarios for [Algames.jl](https://github.com/simon-lc/Algames.jl).

## Installation
```
Pkg.add("AlgamesDriving")
```
## Related Paper
[RSS 2020](http://www.roboticsproceedings.org/rss16/p091.pdf)

## Autonomous Robots paper submission:
The experiments are available [here](https://github.com/simon-lc/AlgamesDriving.jl/tree/autonomous_robots_2021_revisions/autonomous_robots_figures).

## Quick Start
To run a simple example of a autonomous driving see script in /examples/intro_example.jl.
- Install AlgamesDriving: `add AlgamesDriving#v0.1.4`
- Install Algames: `add Algames#v0.1.6`
- Install Plots: `add Plots`
- Add other dependencies: `add LinearAlgebra Meshcat StaticArrays`
- run the script!

## Starter Code
Generate Autonomous Driving Scenario
```
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
```
Generate and Solve the Dynamic Game Problem
```
N = 20 # number of time steps
dt = 0.1 # time step
solver_opts = Options() # solver options
prob = GameProblem(N, dt, sce, solver_opts) # atonomous driving problem
@time newton_solve!(prob) # solve the problem

# Plotting vehicles' trajectories and the solver's progress
plot!(model, prob.pdtraj.pr)
plot!(prob.stats)
```
 Visualize the Results
```
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

```

![alt text](https://github.com/simon-lc/Algames.jl/blob/master/readme_banner.jpeg?raw=true)
