################################################################################
# Toy Example
################################################################################
using Plots
T = Float64

# Define the dynamics of the system
p = 2 # Number of players
model = DoubleIntegratorGame(p=p) # game with 3 players with unicycle dynamics
# model = UnicycleGame(p=p) # game with 3 players with unicycle dynamics
# model = BicycleGame(p=p) # game with 3 players with unicycle dynamics
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
radius = 0.5*ones(p)
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
opts.αx_dual = [sqrt(10), 1/sqrt(10)]
s = 0.1
opts.αx_dual = [sqrt(s), 1/sqrt(s)]
# opts.αx_dual = [1/sqrt(10), sqrt(10)]
# Define the game problem
prob = GameProblem(N,dt,x0,model,opts,game_obj,game_con)

prob.game_con.state_conval[1][1]
# Solve the problem
@time newton_solve!(prob)
# @profiler newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)

residual!(prob, prob.pdtraj)
# regularize_residual!(prob.core, prob.opts, prob.pdtraj, prob.pdtraj)
residual_jacobian!(prob, prob.pdtraj)
# regularize_residual_jacobian!(prob)
prob.core.res

Δtraj = - \(lu(prob.core.jac), prob.core.res)
set_traj!(prob.core, prob.Δpdtraj, Δtraj)
prob.Δpdtraj
Δ_step(prob.Δpdtraj, 1.0)

vecs = eigvecs(Matrix(prob.core.jac))
vals = eigvals(Matrix(prob.core.jac))
vals_norm = norm.(vals)
min_val = minimum(vals_norm)
i_min = argmin(vals_norm)
min_vec = vecs[:,i_min]

mean(norm.((prob.core.jac*min_vec - min_val*min_vec)))

function get_min_eig(prob::GameProblem)
    residual!(prob, prob.pdtraj)
    res = eigs(prob.core.jac, which=:SM, nev=1)
    min_val = res[1]
    min_vec = res[2]

    return res[1:2]
end


function evolve(prob::GameProblem, dt::T) where {T}
    eig_pdtraj = PrimalDualTraj(prob.probsize, dt)
    for j = 1:10
        min_val, min_vec = get_min_eig(prob)
        # @show min_val
        residual!(prob)
        @show mean(abs.(prob.core.res))
        set_traj!(prob.core, eig_pdtraj, real.(min_vec)[:,1])
        scale = mean(norm.(real.(min_vec)))
        α = 1e-2/scale
        update_traj!(prob.pdtraj, prob.pdtraj, α, eig_pdtraj)
        prob.opts.shift = 0
        newton_solve!(prob)
        plot!(prob.model, prob.pdtraj.pr)
    end
    return nothing
end

prob_copy = deepcopy(prob)
evolve(prob_copy, dt)

prob.game_con.state_conval[1]
prob.game_con.state_conval[1][1].λ
prob.game_con.state_conval[1][1].vals
prob.game_con.state_conval[1][1].jac[1]
prob.game_con.state_conval[1][1].con
prob.game_con.state_conval[2][1].λ
prob.game_con.state_conval[2][1].vals
prob.game_con.state_conval[2][1].con

eigs(prob.core.jac, which=:LM)
res = eigs(prob.core.jac, which=:SM, nev=1)
res[1]
res[2]
min_vec + res[2]


size(prob.core.jac)
mat = Matrix(log.(10, abs.(prob.core.jac)))
heatmap(mat, color = :greys) #which will also give you a color legend
heatmap(mat[190:290,1:100], color = :greys) #which will also give you a color legend

# kick out of inner loop if not progress made
# add probsize to game_con struct
# add constructor for multiple radius of collision


function traj_dist(prob::GameProblem ;M::Int=10)
    traj = []
    for j = 1:M
        prob.opts.seed = j
        newton_solve!(prob)
        deepcopy(prob.pdtraj.pr)
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
            xi = [state(traj[j][k])[model.pz[i][1]] for k=1:N]
            yi = [state(traj[j][k])[model.pz[i][2]] for k=1:N]
            plot!(xi, yi, color=col[i], label=false)
            scatter!(xi, yi, color=col[i], label=false)
        end
    end
    display(plt)
    return nothing
end

function solver_progress(prob::GameProblem; M::Int=10, II::Int=20, OI::Int=20)
    anim = @animate for ii = 1:Int(ceil(II/2)):II
        prob.opts.outer_iter=OI
        prob.opts.inner_iter=ii
        traj_dist(prob, M=M)
    end
    gif(anim, "Documents/solver_progress_con.gif", fps = 2)
    return nothing
end

prob.opts.amplitude_init = 1e1
prob.opts.f_init = x -> (rand(x) .- 0.5)
prob.opts.outer_iter = 0
prob.opts.inner_iter = 0
traj_dist(prob, M=100)

prob_copy = deepcopy(prob)
traj_dist(prob_copy, M=100)
solver_progress(prob_copy, M=50, II=20, OI=20)
newton_solve!(prob_copy)
prob_copy.stats



# function balance_dual_update!(game_con::GameConstraintValues, game_obj::GameObjective,
#     pdtraj::PrimalDualTraj)
#     probsize = game_obj.probsize
#     N = probsize.N
#     p = probsize.p
#     pu = probsize.pu
#
#     cost_gradient!(game_obj, pdtraj)
#     @show 111
#     for i = 1:p
#         for k = 2:N
#             n_obj = length(game_obj.E[i])
#             for j = 1:n_obj
#                 ∇x_Jik = game_obj.E[i][j].cost[k].q
#                 n_con = length(game_con.state_conlist[i].constraints)
#                 for l = 1:n_con
#                     conval = game_con.state_conval[i][l]
#                     TrajectoryOptimization.evaluate!(conval, pdtraj.pr)
#                     TrajectoryOptimization.jacobian!(conval, pdtraj.pr)
#                     TrajectoryOptimization.cost_expansion!(conval)
#                     ∇x_Cik = conval.jac[k-1]# need to use enumarate to coincide k+1 and k
#                     @show i
#                     @show -(∇x_Cik*∇x_Cik')*∇x_Cik*∇x_Jik
#                 end
#             end
#         end
#     end
#
#     return nothing
# end


colmult = CollisionMultiplier(probsize)
get_collision_multiplier!(colmult, prob.game_con)
get_balance!(colmult, prob.game_con, prob.pdtraj)
balance_dual!(colmult, prob.game_con)
unbalance_dual!(colmult, prob.game_con)


prob.game_con.state_conval[1]

balance_dual_update!(prob.game_con, prob.game_obj, prob.pdtraj)

prob.game_obj



prob.game_con.state_conlist[1]
prob.game_con.state_conval[1][1]



function residual2!(prob::GameProblem{KN,n,m,T,SVd,SVx}, pdtraj::PrimalDualTraj{KN,n,m,T,SVd}) where {KN,n,m,T,SVd,SVx}
	N = prob.probsize.N
	p = prob.probsize.p
    pu = prob.probsize.pu
	core = prob.core
	model = prob.model
	game_obj = prob.game_obj

	# Initialization
	prob.core.res .= 0.0
    stamp = VStamp()
	∇dyn = zeros(MMatrix{n,(n+m),T,n*(n+m)})

	# Cost
	cost_gradient!(game_obj, pdtraj)
	for i = 1:p
		# State cost
		for k = 1:N
			stampify!(stamp, :opt, i, :x, 1, k)
			n_obj = length(game_obj.E[i])
			for j = 1:n_obj
				valid(stamp, N, p) ? add2sub(core.res_sub[stamp], game_obj.E[i][j].cost[k].q) : nothing
			end
		end
		# Control Cost
		for k = 1:N-1
			stampify!(stamp, :opt, i, :u, i, k)
			n_obj = length(game_obj.E[i])
			for j = 1:n_obj
				valid(stamp, N, p) ? add2sub(core.res_sub[stamp], game_obj.E[i][j].cost[k].r[pu[i]]) : nothing
			end
		end
	end
	# # Dynamics penalty
	# for k = 1:N-1
	# 	∇dynamics!(∇dyn, model, pdtraj, k)
	# 	for i = 1:p
	# 		λik = pdtraj.du[i][k]
	# 		stampify!(stamp, :opt, i, :x, 1, k)
	# 		valid(stamp, N, p) ? add2sub(core.res_sub[stamp], ∇dyn[:,core.dyn[:x][1]]'*λik) : nothing
	# 		stampify!(stamp, :opt, i, :u, i, k)
	# 		valid(stamp, N, p) ? add2sub(core.res_sub[stamp], ∇dyn[:,core.dyn[:u][i]]'*λik) : nothing
	# 		stampify!(stamp, :opt, i, :x, 1, k+1)
	# 		valid(stamp, N, p) ? add2sub(core.res_sub[stamp], -λik) : nothing
	# 	end
	# end

	# # Constraints
	# constraint_residual!(prob, pdtraj)

	# # Dynamics
    # for k = 1:N-1
    #     stampify!(stamp, :dyn, 1, :x, 1, k)
    #     valid(stamp, N, p) ? add2sub(core.res_sub[stamp], dynamics_residual(model, pdtraj, k)) : nothing # shouldn't allocate
    # end
    return nothing
end



prob.opts.γ
