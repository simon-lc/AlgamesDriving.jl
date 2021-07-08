################################################################################
# Toy Example
################################################################################
using Plots
T = Float64

# Define the dynamics of the system
p = 3 # Number of players
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
Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p] # Quadratic state cost
R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2, 0.0,0,0]),
      SVector{model.ni[2],T}([2, 0.0,0,0]),
      SVector{model.ni[3],T}([2, 0.0,0,0]),
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
    0.0, 0.0, 0.0, #0.0,
   -0.4, 0.0, 0.4, #0.6,
    0.0, 0.0, 0.0, #0.0,
    0.0, 0.0, 0.0, #0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-5
opts.α_dual = 1.0
opts.λ_max = 1.0*1e7
opts.ϵ_dyn = 1e-5
opts.ϵ_sta = 1e-5
opts.ϵ_con = 1e-5
opts.ϵ_opt = 1e-5
opts.regularize = true
opts.αx_dual = ones(p)

# s = 0.5
# opts.αx_dual = [sqrt(s), 1/sqrt(s)]
# Define the game problem
prob = GameProblem(N,dt,x0,model,opts,game_obj,game_con)

# Solve the problem
@time newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)






function kkt_residual(prob::GameProblem)
    residual!(prob, prob.pdtraj)
    col_res = collision_residual(prob)
    kkt_res = vcat(prob.core.res, col_res)
    return kkt_res
end

function collision_residual(prob::GameProblem)
    probsize = prob.probsize
    N = probsize.N
    p = probsize.p
    px = probsize.px
    n_col = Int(p*(p-1)/2)
    col_res = zeros((N-1)*n_col)

    off = 1
    for k = 2:N #fix for correct indexing
        for i = 1:p
            for j = i+1:p
                ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
                conval = prob.game_con.state_conval[i][ind]
                col_res[off] = conval.vals[k-1][1]
                off += 1
            end
        end
    end
    return col_res
end


function kkt_residual_jacobian(prob::GameProblem)
    probsize = prob.probsize
    N = probsize.N
    p = probsize.p
    px = probsize.px
    n_col = Int(p*(p-1)/2)
    n_λ = p*(p-1)
    n_res = length(prob.core.res)

    residual_jacobian!(prob, prob.pdtraj)
    kkt_jac = spzeros(n_res + (N-1)*n_col, n_res + (N-1)*n_λ)
    kkt_jac[1:n_res, 1:n_res] .= prob.core.jac
    off = 1
    for k = 2:N
        for i = 1:p
            stamp = stampify(:opt, i, :x, 1, k)
            vind = prob.core.verti_inds[stamp]
            for j ∈ setdiff(1:p,i)
                ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
                conval = prob.game_con.state_conval[i][ind]
                hind = n_res + off
                kkt_jac[vind, hind:hind] .= conval.jac[k-1]'
                off += 1
            end
        end
    end

    off = 1
    for k = 2:N
        for i = 1:p
            hind = prob.core.horiz_inds[:x][1][k]
            for j = i+1:p
                ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
                conval = prob.game_con.state_conval[i][ind]
                vind = n_res + off
                kkt_jac[vind:vind, hind] .= conval.jac[k-1]
                off += 1
            end
        end
    end
    return kkt_jac
end

function update_λcol!(prob::GameProblem, Δλcol)
    probsize = prob.probsize
    N = probsize.N
    p = probsize.p
    px = probsize.px
    off = 1
    for k = 2:N
        for i = 1:p
            for j ∈ setdiff(1:p,i)
                ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
                conval = prob.game_con.state_conval[i][ind]
                # @show off
                # @show "before" conval.λ[k-1][1]
                if conval.λ[k-1][1] != 0
                    conval.λ[k-1][1] += Δλcol[off]
                    conval.λ[k-1][1] = max(0, conval.λ[k-1][1])
                end
                # @show "after " conval.λ[k-1][1]
                off += 1
            end
        end
    end
    return nothing
end

function display_nullspace(prob_; M::Int=5, amplitude::T=1e-3, atol::T=1e-10) where {T}
    kkt_jac = kkt_residual_jacobian(prob_)
    kkt_jac = Matrix(kkt_jac)
    n_res = length(prob_.core.res)
    ns = nullspace(kkt_jac, atol=atol)
    n_vec = size(ns)[2]
    nv = ns[:,rand(1:n_vec)]
    nv = nv./mean(abs.(nv))*amplitude
    plt = plot(legend=false)
    anim = @animate for k = 1:M
        prob = deepcopy(prob_)
        Δtraj = nv[1:n_res]
        Δλcol = nv[n_res+1:end]
        set_traj!(prob.core, prob.Δpdtraj, k*Δtraj)
        # update_traj!(prob.pdtraj, prob.pdtraj, 1.0, prob.Δpdtraj)
        update_λcol!(prob, k*Δλcol)
        # @show Δλcol

        prob.opts.shift = 0
        prob.opts.α_dual = 0.0
        prob.opts.αx_dual = zeros(p)
        prob.opts.dual_reset = false
        prob.opts.ρ_0 = prob.pen.ρ[1]
        prob.opts.ρ_max = prob.pen.ρ[1]
        prob.opts.inner_print = false
        prob.opts.outer_print = false
        prob.opts.inner_iter = 20
        prob.opts.outer_iter = 3
        prob.opts.reg_0 = 1e-8
        newton_solve!(prob)
        residual!(prob)
        @show mean(abs.(prob.core.res))
        # @show mean(abs.(Δtraj))
        # @show mean(abs.(Δλcol))
        opt = mean(abs.(prob.stats.opt_vio[end].vio))
        sta = mean(abs.(prob.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
        println("opt = "*scn(opt)*"  ", opt <= prob.opts.ϵ_opt )
        println("sta = "*scn(sta)*"  ", opt <= prob.opts.ϵ_opt )
        println("dyn = "*scn(dyn)*"  ", opt <= prob.opts.ϵ_opt )
        # plot!(prob.stats)
        if opt < 1e-5
            plot!(prob.model, prob.pdtraj.pr, plt=plt)
        end
        # plot!(prob.stats)
    end
    path = joinpath("~/Documents/nullspace_p3_din.gif")
    gif(anim, path, fps = 10)
    return nothing
end


function follow_nullspace(prob; M::Int=5, amplitude::T=1e-3, atol::T=1e-10) where {T}

    plt = plot(legend=false)
    anim = @animate for k = 1:M
        Random.seed!(k)
        kkt_jac = kkt_residual_jacobian(prob)
        kkt_jac = Matrix(kkt_jac)
        n_res = length(prob.core.res)
        ns = nullspace(kkt_jac, atol=atol)
        n_vec = size(ns)[2]
        nv = ns[:,10]
        # nv = ns[:,rand(1:n_vec)]
        nv = nv./mean(abs.(nv))*amplitude

        Δtraj = nv[1:n_res]
        Δλcol = nv[n_res+1:end]
        set_traj!(prob.core, prob.Δpdtraj, Δtraj)
        update_traj!(prob.pdtraj, prob.pdtraj, 1.0, prob.Δpdtraj)
        update_λcol!(prob, Δλcol)

        prob.opts.shift = 0
        prob.opts.α_dual = 0.0
        prob.opts.αx_dual = zeros(p)
        prob.opts.dual_reset = false
        prob.opts.ρ_0 = prob.pen.ρ[1]
        prob.opts.ρ_max = prob.pen.ρ[1]
        prob.opts.inner_print = false
        prob.opts.outer_print = false
        prob.opts.inner_iter = 20
        prob.opts.outer_iter = 10
        prob.opts.reg_0 = 1e-8
        newton_solve!(prob)
        residual!(prob)
        println("***********************")
        opt = mean(abs.(prob.stats.opt_vio[end].vio))
        sta = mean(abs.(prob.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
        println("opt = "*scn(opt)*"  ", opt <= prob.opts.ϵ_opt )
        println("sta = "*scn(sta)*"  ", opt <= prob.opts.ϵ_opt )
        println("dyn = "*scn(dyn)*"  ", opt <= prob.opts.ϵ_opt )
        if opt < 1e-5
            plot!(prob.model, prob.pdtraj.pr, plt=plt)
        end
        # plot!(prob.stats)
    end
    path = joinpath("~/Documents/follow_nullspace_p3_uni.gif")
    gif(anim, path, fps = 20)
    return nothing
end

# evaluate!(prob.game_con, prob.pdtraj.pr)
# collision_residual(prob)
# kkt_residual(prob)
# kkt_jac = kkt_residual_jacobian(prob)
# ns = nullspace(Matrix(kkt_jac), atol=1e-10)
# nv = ns[:,1]

prob_copy = deepcopy(prob)
# display_nullspace(prob_copy, M=10, amplitude=1e-2)
follow_nullspace(prob_copy, M=10, amplitude=1e-4)
