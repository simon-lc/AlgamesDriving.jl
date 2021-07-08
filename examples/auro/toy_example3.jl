################################################################################
# Toy Example
################################################################################
T = Float64

# Define the dynamics of the system
p = 2 # Number of players
# model = DoubleIntegratorGame(p=p) # game with 3 players with unicycle dynamics
model = UnicycleGame(p=p) # game with 3 players with unicycle dynamics
model = BicycleGame(p=p) # game with 3 players with unicycle dynamics
n = model.n
m = model.m

# Define the horizon of the problem
N = 20 # N time steps
dt = 0.1 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(10*ones(SVector{model.ni[i],T})) for i=1:p] # Quadratic state cost
R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2,+0.4,0,0]),
      SVector{model.ni[2],T}([2, 0.0,0,0]),
      # SVector{model.ni[3],T}([3,-0.4,0,0]),
      # SVector{model.ni[4],T}([3,+0.8,0,0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]

# # Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.5*ones(p)
μ = 4.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
# # Add collision avoidance
# radius = 0.05
# add_collision_avoidance!(game_con, radius)

# # Add control bounds
# u_max =  5*ones(SVector{m,T})
# u_min = -5*ones(SVector{m,T})
# add_control_bound!(game_con, u_max, u_min)

# # Add wall constraint
# walls = [Wall([-1.0,-1.75], [1.0,0.25], [1.,-1.]/sqrt(2))]
# add_wall_constraint!(game_con, walls)

# # Add circle constraint
# xc = [1., 2., 3.]
# yc = [1., 2., 3.]
# radius = [0.1, 0.2, 0.3]
# add_circle_constraint!(game_con, xc, yc, radius)

# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.0, 0.0,# 0.5, #0.0,
   -0.4, 0.0,# 0.7, #0.6,
    0.0, 0.0,# 0.0, #0.0,
    0.0, 0.0,# 0.0, #0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-8
opts.α_dual = 1.0
opts.λ_max = 1.0*1e7
opts.ϵ_dyn = 1e-3
opts.ϵ_sta = 1e-3
opts.ϵ_con = 1e-3
opts.ϵ_opt = 1e-3
opts.regularize = true
# Define the game problem
prob = Algames.GameProblem(N,dt,x0,model,opts,game_obj,game_con)

# Solve the problem
@time newton_solve!(prob)
# @profiler newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)

prob.stats.outer_iter
# kick out of inner loop if not progress made
# add probsize to game_con struct
# add constructor for multiple radius of collision


function traj_dist(prob::Algames.GameProblem ;M::Int=10)
    traj = []
    for j = 1:M
        prob.opts.seed = j
        newton_solve!(prob)
        push!(traj, deepcopy(prob.pdtraj.pr))
    end
    plotset(prob.model, traj)
    return traj
end

function plotset(model::AbstractGameModel, traj)
    plt = plot(aspect_ratio=:equal)
    M = length(traj)
    N = length(traj[1])
    col = [:blue, :orange, :green]
    for j = 1:M
        for i = 1:model.p
            xi = [Algames.state(traj[j][k])[model.pz[i][1]] for k=1:N]
            yi = [Algames.state(traj[j][k])[model.pz[i][2]] for k=1:N]
            plot!(xi, yi, color=col[i], label=false)
            scatter!(xi, yi, color=col[i], label=false)
        end
    end
    display(plt)
    return nothing
end

function solver_progress(prob::Algames.GameProblem; M::Int=10, II_max::Int=2, IO::Int=1)
    # anim = @animate for ii = II_max:II_max
    anim = @animate for ii = 0:II_max
        prob.opts.outer_iter=IO
        prob.opts.inner_iter=ii
        traj_dist(prob, M=M)
    end
    gif(anim, "Documents/solver_progress_Uni_nncvx.gif", fps = 2)
    return nothing
end

prob.opts.amplitude_init = 3.0
prob.opts.f_init = x -> (rand(x) .- 0.5)
prob.opts.outer_iter = 0
prob.opts.inner_iter = 0
traj_dist(prob, M=100)

solver_progress(prob, M=100, II_max=20, IO=1)
solver_progress(prob, M=100, II_max=20, IO=20)
