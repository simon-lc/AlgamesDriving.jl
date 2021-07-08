################################################################################
# Toy Example
################################################################################
using Plots
T = Float64

# Define the dynamics of the system
p = 2 # Number of players
model = DoubleIntegratorGame(p=p) # game with 3 players with unicycle dynamics
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
# Q = [Diagonal(10*ones(SVector{model.ni[1],T})), Diagonal(1*ones(SVector{model.ni[2],T}))] # Quadratic state cost
# Q = [Diagonal(1*ones(SVector{model.ni[1],T})), Diagonal(10*ones(SVector{model.ni[2],T}))] # Quadratic state cost
R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# R = [Diagonal(0.1*ones(SVector{model.mi[1],T})), Diagonal(0.01*ones(SVector{model.mi[2],T}))] # Quadratic control cost
# R = [Diagonal(0.01*ones(SVector{model.mi[1],T})), Diagonal(0.1*ones(SVector{model.mi[2],T}))] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2, 0.0,0,0]),
      SVector{model.ni[2],T}([2, 0.0,0,0]),
      # SVector{model.ni[3],T}([3,-0.4,0,0]),
      # SVector{model.ni[4],T}([3,+0.8,0,0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]

# # Objectives of the game
game_obj = GameObjective(Q,R,xf,uf,N,model)
# radius = 0.5*ones(p)
# μ = 4.0*ones(p)
# μ = 4.0*[1.0, 0.1]
# μ = 4.0*[0.1, 1.0]
# add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = GameConstraintValues(probsize)
# Add collision avoidance
radius = 0.19
add_collision_avoidance!(game_con, radius)

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
   -0.2, 0.2,# 0.7, #0.6,
    0.0, 0.0,# 0.0, #0.0,
    0.0, 0.0,# 0.0, #0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-7
opts.α_dual = 1.0
opts.λ_max = 1.0*1e7
opts.ϵ_dyn = 1e-6
opts.ϵ_sta = 1e-6
opts.ϵ_con = 1e-6
opts.ϵ_opt = 1e-6
opts.regularize = true
# opts.αx_dual = ones(p)
# opts.αx_dual = [sqrt(10), 1/sqrt(10)]
# opts.αx_dual = [1/sqrt(10), sqrt(10)]

s = 0.5
opts.αx_dual = [sqrt(s), 1/sqrt(s)]
# Define the game problem
prob = GameProblem(N,dt,x0,model,opts,game_obj,game_con)

prob.game_con.state_conval[1][1]
# Solve the problem
@time newton_solve!(prob)
# @profiler newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)

function subspace_animation(prob::GameProblem)
    p = prob.probsize.p
    S = exp.(Vector(-1.0:0.05:1.0) .* log(10))
    plt = plot(legend=false)

    anim = @animate for s in S
        @show s
        prob.opts.αx_dual = [sqrt(s), 1/sqrt(s)]
        set_constraint_params!(prob.game_con, prob.opts)
        newton_solve!(prob)
        plot!(prob.model, prob.pdtraj.pr, plt=plt)
        text(plt,4, 0.5, "hello")
        # plot!(prob.stats)
    end
    path = joinpath("~/Documents/p2_subspace_din.gif")
    gif(anim, path, fps = 10)
    return nothing
end

subspace_animation(prob)
