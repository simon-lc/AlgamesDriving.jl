################################################################################
# Toy Example
################################################################################
using Plots
T = Float64

# Define the dynamics of the system
p = 3 # Number of players
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
# @profiler newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)



function update_λ!(ascore::ActiveSetCore, prob::GameProblem, Δλcol::AbstractVector)
    probsize = prob.probsize
    N = probsize.N
    p = probsize.p
    px = probsize.px


    stamp = CStamp()
    for i = 1:p
        for j ∈ setdiff(1:p,i)
            conval, v = get_collision_conval(prob.game_con, i, j)
            if v
                for (l,k) in enumerate(conval.inds)
                    stampify!(stamp, :h, :col, i, j, k)
                    if conval.active[l][1] == 1
                        conval.λ[l] += Δλcol[horizontal_idx(ascore, stamp) .- probsize.S]
                        conval.λ[l] = max.(0, conval.λ[l])
                    end
                end
            end
        end
    end
    return nothing
end

function subspace_dimension(prob::GameProblem; α::T=2e-3) where {T}
    prob_ = deepcopy(prob)
    probsize = prob.probsize
    ascore = ActiveSetCore(probsize)
    update_nullspace!(ascore, prob, prob.pdtraj)
    ns = length(ascore.null.vec)
    val = 0
    subspace = Vector{PrimalDualTraj}([deepcopy(prob.pdtraj)])
    plt = plot(legend=false)
    anim = @animate for l = 1:ns
        prob_ = deepcopy(prob)
        prob_.opts.shift = 0
        prob_.opts.α_dual = 0.0
        prob_.opts.αx_dual = 1e-2*ones(p)
        # prob_.opts.αx_dual = 0.0*ones(p)
        prob_.opts.dual_reset = false
        prob_.opts.ρ_0 = prob_.pen.ρ[1]
        prob_.opts.ρ_max = prob_.pen.ρ[1]
        prob_.opts.inner_print = false
        prob_.opts.outer_print = false
        prob_.opts.inner_iter = 20
        # prob_.opts.inner_iter = 20
        prob_.opts.outer_iter = 20
        # prob_.opts.outer_iter = 3
        prob_.opts.reg_0 = 1e-5
        # ϵ = 1e-6
        ϵ = 1e-5
        prob_.opts.ϵ_opt = ϵ
        prob_.opts.ϵ_dyn = ϵ
        prob_.opts.ϵ_con = ϵ
        prob_.opts.ϵ_sta = ϵ
        residual!(prob_)
        set_traj!(prob_.core, prob_.Δpdtraj, α*ascore.null.Δtraj[l])
        update_traj!(prob.pdtraj, prob_.pdtraj, 1.0, prob_.Δpdtraj)
        update_λ!(ascore, prob_, α*ascore.null.Δλ[l])
        newton_solve!(prob_)
        plot!(prob_.stats)

        opt = mean(abs.(prob_.stats.opt_vio[end].vio))
        sta = mean(abs.(prob_.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob_.stats.dyn_vio[end].vio))
        con = mean(abs.(prob_.stats.con_vio[end].vio))
        opts = prob_.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            val += 1
            plot!(prob_.model, prob_.pdtraj.pr, plt=plt)
            push!(subspace, deepcopy(prob_.pdtraj))
        end
    end
    @show ns
    @show val
    path = joinpath("~/Documents/nullspace_dim_p3_$(string(typeof(model).name)).gif")
    gif(anim, path, fps = 10)
    return ascore, subspace
end


function pca(prob::GameProblem, subspace::Vector{PrimalDualTraj})
    probsize = prob.probsize
    N = probsize.N
    n = probsize.n
    m = probsize.m
    p = probsize.p
    px = probsize.px
    S = probsize.S
    M = length(subspace)-1

    V = []
    v = zeros(S)
    v_ref = zeros(S)
    get_traj!(prob.core, subspace[1], v_ref)
    for k = 1:M
        get_traj!(prob.core, subspace[k+1], v)
        v = v - v_ref
        push!(V, deepcopy(v))
    end


    # R = (N-1)*(n+m)
    pr_mask = primals_mask(prob.core, probsize)
    R = length(pr_mask)
    W = [v[pr_mask] for v in V]
    # W = [v[1:S] for v in V]
    m = sum(W) ./ M
    Wc = [w .- m for w in W]
    cov = sum([wc*wc' for wc in Wc])/M
    vecs = eigvecs(cov)
    vals = eigvals(cov)
    eigs = [(vecs[:,i], vals[i]) for i=1:R]
    # eigs = [(vecs[:,i], vals[i]) for i=1:S]
    sort!(eigs, by = x -> x[2], rev=true)
    vecs = [e[1] for e in eigs[1:M]]
    vals = [e[2] for e in eigs[1:M]]

    @show round.(log.(10, abs.(vals)), digits=3)


    plt = plot(legend=false)
    pdtraj_ref = subspace[1]
    plot!(prob.model, pdtraj_ref.pr, plt=plt)

    pdtraj_1 = deepcopy(pdtraj_ref)
    v1 = zeros(S)
    v1[pr_mask] = vecs[1]
    set_traj!(prob.core, pdtraj_1, v1)

    pdtraj_2 = deepcopy(pdtraj_ref)
    v2 = zeros(S)
    v2[pr_mask] = vecs[2]
    set_traj!(prob.core, pdtraj_2, v2)

    pdtraj_3 = deepcopy(pdtraj_ref)
    v3 = zeros(S)
    v3[pr_mask] = vecs[3]
    set_traj!(prob.core, pdtraj_3, v3)

    x = [[state(pdtraj_ref.pr[k])[px[i][1]] for k=2:N] for i=1:p]
    y = [[state(pdtraj_ref.pr[k])[px[i][2]] for k=2:N] for i=1:p]
    u1 = [[state(pdtraj_1.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v1 = [[state(pdtraj_1.pr[k])[px[i][2]]  for k=2:N] for i=1:p]
    u2 = [[state(pdtraj_2.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v2 = [[state(pdtraj_2.pr[k])[px[i][2]]  for k=2:N] for i=1:p]
    u3 = [[state(pdtraj_3.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v3 = [[state(pdtraj_3.pr[k])[px[i][2]]  for k=2:N] for i=1:p]
    β = 2.0
    for i = 1:p
        quiver!(x[i],y[i],quiver=(β*u1[i],β*v1[i]), linewidth=1.5, color=:cyan)
        quiver!(x[i],y[i],quiver=(β*u2[i],β*v2[i]), linewidth=1.5, color=:red)
        quiver!(x[i],y[i],quiver=(β*u3[i],β*v3[i]), linewidth=1.5, color=:black)
    end
    # y = rand(1:10,N)
    # u = rand(N)
    # v = rand(N)
    # scatter(x,y)

    display(plt)

    return vals, pdtraj_1
end

function primals_mask(core::NewtonCore, probsize::ProblemSize)
    N = probsize.N
    inds = Vector{Int}()
    stamp = HStamp()
    for k = 1:N-1
        # States
        stampify!(stamp, :x, 1, k+1)
        ind = horizontal_idx(core, stamp)
        inds = vcat(inds, Vector(ind))
        # Controls
        for i = 1:p
            stampify!(stamp, :u, i, k)
            ind = horizontal_idx(core, stamp)
            inds = vcat(inds, Vector(ind))
        end
    end
    return inds
end

function display_eigvals(vals)
    plt = plot(legend=false)
    plot!(vals,
        linetype=:steppost,
        linewidth=3.0,
        title="PCA on the Nash Equilibrium subspace \n around a nominal NE trajectory.",
        xlabel="eigval_index",
        ylabel="eigval_magnitude")
    display(plt)
    return nothing
end

prob_copy = deepcopy(prob)
# ascore, subspace = subspace_dimension(prob_copy, α=4e-3) # Unicycle, # Bicycle
subspace
vals, pdtraj_1 = pca(prob, subspace)
display_eigvals(vals)

pdtraj_1

pr_mask = primals_mask(prob.core, probsize)
