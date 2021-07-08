vis = Visualizer()
open(vis)

# Create Roadway
roadway_opts = MergingRoadwayOptions(merging_point=1.4, lane_width=0.28)
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 3
model = BicycleGame(p=p)
n = model.n
m = model.m

# Define the horizon of the problem
N = 15 # N time steps
dt = 0.10 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(3*SVector{model.ni[i],T}([1., 1.2, 1., 1.])) for i=1:p] # Quadratic state cost
R = [Diagonal(0.3*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2.0, 0.05, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.05, 0.0, 0]),
      SVector{model.ni[2],T}([2.0, 0.05, 0.0, 0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
# radius = 0.22*ones(p)
# μ = 40.0*ones(p)
# add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
radius = 0.08
add_collision_avoidance!(game_con, radius)


# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.2,     0.2,    0.2,
   -0.45,   -0.13,   0.13,
    0.45,    0.60,   0.60,
    0.15,    0.0,    0.0,
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
opts.ϵ_dyn = 1e-6
opts.ϵ_sta = 1e-6
opts.ϵ_con = 1e-6
opts.ϵ_opt = 1e-6
opts.regularize = true
# Define the game problem
prob = Algames.GameProblem(N,dt,x0,model,opts,game_obj,game_con)

# Solve the problem
newton_solve!(prob)

plot!(prob.model, prob.pdtraj.pr)
plot!(prob.stats)

players = Vector{Player{T}}(undef, p)
player_opts = HighwayPlayerOptions{model.ni[1],model.mi[1],T}()
players[1] = Player(model, roadway.lane[3])
players[2] = Player(model, roadway.lane[3])
players[3] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)

# Initialize visualizers
# vis = Visualizer()
# open(vis)

# Visualize trajectories
set_scenario!(vis, sce)
set_env!(vis, VehicleState(1.3, 0.0, 0.0, 0.0))
set_camera_birdseye!(vis, height=5.0)

build_waypoint!(vis, sce.player, N, key=0)
set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr, key=0)
set_traj!(vis, model, sce, prob.pdtraj.pr)


get_num_active_constraint(prob)




include("explorer.jl")
prob_copy = deepcopy(prob)
ascore = ActiveSetCore(probsize)
active_vertical_mask!(ascore, prob_copy.game_con)
vmask = deepcopy(ascore.vmask)

# Tests
Sh = probsize.S+p*(p-1)*(N-1)
Sv = probsize.S+Int(p*(p-1)*(N-1)/2)
update_nullspace!(ascore, prob_copy, prob_copy.pdtraj)
ns = length(ascore.null.vec)

v = deepcopy(ascore.null.vec[2])
β = 1.0
α = 1e-2
λ = 1e-10*rand(Sv)
x = get_primal_dual(ascore, prob_copy)

prob_copy2 = deepcopy(prob)
prob_copy3 = deepcopy(prob)
maximum(abs.(α*v))
plot!(model, prob_copy.pdtraj.pr)

xp, λ = newton_solver(α, λ, x, v, β, ascore, prob_copy2, vmask)

plot!(model, prob_copy.pdtraj.pr)
plot!(model, prob_copy2.pdtraj.pr)
set_primal_dual!(ascore, prob_copy3, x+α*v)
plot!(model, prob_copy3.pdtraj.pr)





function lloo(prob::Algames.GameProblem)
	ascore = ActiveSetCore(probsize)
	update_nullspace!(ascore, prob, prob.pdtraj)
	ns = length(ascore.null.vec)
	x0 = get_primal_dual(ascore, prob)
	v0 = deepcopy(ascore.null.vec[2])

	costs = []
 	pdtrajs = []

	plt = plot()
	for l = 1:100
		ascore = ActiveSetCore(probsize)
		active_vertical_mask!(ascore, prob.game_con)
		vmask = deepcopy(ascore.vmask)
		na = get_num_active_constraint(prob)
		@show na
		# # Tests
		# update_nullspace!(ascore, prob, prob.pdtraj)
		# ns = length(ascore.null.vec)
		# v = deepcopy(ascore.null.vec[2])

		β = 1.0
		α = 1e-3
		λ = 1e-10*rand(Sv)
		x = get_primal_dual(ascore, prob)

		xp, λ = simple(α, λ, x, v0, β, ascore, prob, vmask)
		# xp, λ = newton_solver(α, λ, x, v0, β, ascore, prob, vmask)
		set_primal_dual!(ascore, prob, xp)
		push!(costs, cost(prob.game_obj, prob.pdtraj))
		push!(pdtrajs, deepcopy(prob.pdtraj))

		plot!(model, prob.pdtraj.pr, plt=plt)
		prob_c = deepcopy(prob)
		set_primal_dual!(ascore, prob_c, x+α*v)
		# plot!(model, prob_c.pdtraj.pr)
	end
	display(plt)

	set_primal_dual!(ascore, prob, x0)
	plt = plot()
	plot!(model, prob.pdtraj.pr, plt=plt)
	set_primal_dual!(ascore, prob, x0+100*1e-3*v0)
	plot!(model, prob.pdtraj.pr, plt=plt)
	display(plt)
	plt2 = plot()
	plot!([c[1] for c in costs], label="1")
	plot!([c[2] for c in costs], label="2")
	plot!([c[3] for c in costs], label="3")
	display(plt2)
	return prob, costs, pdtrajs
end


include("explorer.jl")
prob_copy4 = deepcopy(prob)
prob_copy4, costs, pdtrajs = lloo(prob_copy4)

for (k,pdtraj) in enumerate(pdtrajs)
	build_waypoint!(vis, sce.player, N, key=k)
	set_waypoint_traj!(vis, model, sce, pdtraj.pr, key=k)
end

plt2 = plot()
plot!([c[1] for c in costs], label="1")
plot!([c[2] for c in costs], label="2")
plot!([c[3] for c in costs], label="3")
display(plt2)

a = 10
a = 10
a = 10
a = 10
a = 10
a = 10

Algames.evaluate!(game_obj)







function cost(game_obj::Algames.GameObjective, pdtraj::PrimalDualTraj)
	probsize = game_obj.probsize
	N = probsize.N
	p = probsize.p
	p = probsize.p
	c = zeros(p)
	for i = 1:p
		for obj in game_obj.obj[i]
			c[i] = Algames.cost(obj, pdtraj.pr)
		end
	end
	return c
end

cost(prob_copy4.game_obj, prob_copy4.pdtraj)
prob_copy4.game_obj

a = 10
a = 10
a = 10
a = 10
a = 10
a = 10


















#
# include("explorer.jl")
# prob_copy = deepcopy(prob)
# # plot!(model, prob_copy.pdtraj.pr)
# ascore = ActiveSetCore(probsize)
# active_vertical_mask!(ascore, prob_copy.game_con)
# vmask = deepcopy(ascore.vmask)
# # set_primal_dual!(ascore, prob_copy, x)
#
# # Tests
# Sh = probsize.S+p*(p-1)*(N-1)
# Sv = probsize.S+Int(p*(p-1)*(N-1)/2)
# # xr = rand(Sh)
# # set_primal_dual!(ascore, prob_copy, xr)
# # xr_ = get_primal_dual(ascore, prob_copy)
# # @test xr == xr_
#
# xp = get_primal_dual(ascore, prob_copy)
# α = 0.2
# x = get_primal_dual(ascore, prob_copy)
# update_nullspace!(ascore, prob_copy, prob_copy.pdtraj)
# ns = length(ascore.null.vec)
# v = deepcopy(ascore.null.vec[1])
# β = 10.0
# # active_vertical_mask!(ascore, prob_copy.game_con)
# # vmask = deepcopy(ascore.vmask)
# λ = 1e-1*rand(Sv)
# f(xp, α, x, v, β)
# λ'*c(x, ascore, prob_copy)
# L(xp, α, λ, x, v, β, ascore, prob_copy, vmask)
# grad(xp, α, λ, x, v, β, ascore, prob_copy, vmask)
# jac(xp, α, λ, x, v, β, ascore, prob_copy, vmask)
# newton_solver(λ, x, v, β, ascore, prob_copy, vmask)








prob_copy = deepcopy(prob)
ascore = ActiveSetCore(probsize)
get_primal_dual(ascore, probsize)






prob_copy = deepcopy(prob)
ascore, subspace = subspace_dimension(prob_copy, α=1e-3) # Bicycle
subspace
vals, ref_pdtraj, eig_pdtraj_1, eig_pdtraj_2, = pca(prob, subspace, β=1.2*10^5)
display_eigvals(vals)


β = 0.8*1e5
display_arrow2(prob_copy, sce, ref_pdtraj, vals[1], eig_pdtraj_1, color=:yellow, key=1, β=β)
display_arrow2(prob_copy, sce, ref_pdtraj, vals[2], eig_pdtraj_2, color=:cornflowerblue, key=2, β=β)

a = 10

function display_arrow2(prob::Algames.GameProblem, sce::Scenario, ref_pdtraj::PrimalDualTraj, val::T,
	eig_pdtraj::PrimalDualTraj; vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color=:orange, key::Int=0, β::T=1.2*10^5) where {T}

	probsize = prob.probsize
	N = probsize.N
	α = vis_opts.α
	for k = 2:N
		vs_ref = standardize(prob.model, Algames.state(ref_pdtraj.pr[k]))
		vs_eig = standardize(prob.model, Algames.state(eig_pdtraj.pr[k]))
		for i = 1:p
			player = sce.player[i]
			arr_vis = ArrowVisualizer(vis["env/arrow/arrow_traj$key/player$(player.id)/$k"])
			col = vis_opts.colors[color]
			mat = MeshPhongMaterial(color=RGBA(col, α))

			setobject!(arr_vis, mat)
			settransform!(arr_vis,
				Point(0.0, 0.0, 0.0),
				Vec(β*val*norm([vs_eig[i].x, vs_eig[i].y]), 0.0, 0.0),
				shaft_radius=0.002,
				max_head_radius=0.01)

			t = Translation(vs_ref[i].x, vs_ref[i].y, 0.01)
			r1 = LinearMap(AngleAxis(pi/2, 0.0, 1.0, 0.0))
			r2 = LinearMap(AngleAxis(-atan(vs_eig[i].y,vs_eig[i].x), 1.0, 0.0, 0.0))
			tr = compose(t, compose(r1, r2))
			settransform!(vis["env/arrow/arrow_traj$key/player$(player.id)/$k"], tr)
		end
	end
	return nothing
end



function display_ghost(vis::Visualizer, prob::Algames.GameProblem, sce::Scenario)
    N = prob.probsize.N
    for k ∈ vcat(Vector((1:10:N)),N)
        @show k
        vis_opts = PlayerVisualizationOptions{T}(α=0.3+0.2*k/10)
        set_player!(vis, sce.player, vis_opts=vis_opts, key=k)
        set_state!(vis, prob.model, sce, Algames.state(prob.pdtraj.pr[k]), key=k)
    end
    return nothing
end

function get_num_active_constraint(prob::Algames.GameProblem)
    probsize = prob.probsize
    p = probsize.p
	update_active_set!(prob.game_con, prob.pdtraj.pr)
    na = 0
    for i = 1:p
        for j = i+1:p
            conval, v = get_collision_conval(prob.game_con, i, j)
            if v
				for (l,k) in enumerate(conval.inds)
					na += conval.active[l][1]
				end
			end
        end
    end
    return na
end

function update_λ!(ascore::ActiveSetCore, prob::Algames.GameProblem, Δλcol::AbstractVector)
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

function subspace_dimension(prob::Algames.GameProblem; α::T=2e-3) where {T}
    prob_ = deepcopy(prob)
    probsize = prob.probsize
    ascore = ActiveSetCore(probsize)
    update_nullspace!(ascore, prob, prob.pdtraj)
    ns = length(ascore.null.vec)
    val = 0
    subspace = Vector{PrimalDualTraj}([deepcopy(prob.pdtraj)])
    plt = plot(legend=false)
    for l = 1:ns
        @show l
        prob_ = deepcopy(prob)
        prob_.opts.shift = 0
        prob_.opts.α_dual = 0.0
        prob_.opts.αx_dual = 1e-3*ones(p)
        prob_.opts.dual_reset = false
        prob_.opts.ρ_0 = prob_.pen.ρ[1]
        prob_.opts.ρ_max = prob_.pen.ρ[1]
        prob_.opts.inner_print = false
        prob_.opts.outer_print = false
        prob_.opts.inner_iter = 20
        prob_.opts.outer_iter = 40
        prob_.opts.reg_0 = 1e-6
        ϵ = 1e-6
        prob_.opts.ϵ_opt = ϵ
        prob_.opts.ϵ_dyn = ϵ
        prob_.opts.ϵ_con = ϵ
        prob_.opts.ϵ_sta = ϵ
        set_traj!(prob_.core, prob_.Δpdtraj, α*ascore.null.Δtraj[l])
        update_traj!(prob.pdtraj, prob_.pdtraj, 1.0, prob_.Δpdtraj)
        update_λ!(ascore, prob_, α*ascore.null.Δλ[l])
        newton_solve!(prob_)
        # plot!(prob_.stats)

        opt = mean(abs.(prob_.stats.opt_vio[end].vio))
        sta = mean(abs.(prob_.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob_.stats.dyn_vio[end].vio))
        con = mean(abs.(prob_.stats.con_vio[end].vio))
        opts = prob_.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            val += 1
			@show val
            # plot!(prob_.model, prob_.pdtraj.pr, plt=plt)
            push!(subspace, deepcopy(prob_.pdtraj))
        end
    end
    @show ns
    @show val
    return ascore, subspace
end

function pca(prob::Algames.GameProblem, subspace::Vector{PrimalDualTraj}; β::T=2.0) where {T}
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

    pr_mask = primals_mask(prob.core, probsize)
	R = length(pr_mask)
	W = [v[pr_mask] for v in V]
    m = sum(W) ./ M
    Wc = [w .- m for w in W]
    cov = sum([wc*wc' for wc in Wc])/(M-1)
    vecs = eigvecs(cov)
    vals = eigvals(cov)
    eigs = [(vecs[:,i], vals[i]) for i=1:R]
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

    x = [[Algames.state(pdtraj_ref.pr[k])[px[i][1]] for k=2:N] for i=1:p]
    y = [[Algames.state(pdtraj_ref.pr[k])[px[i][2]] for k=2:N] for i=1:p]
    u1 = [[vals[1]*Algames.state(pdtraj_1.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v1 = [[vals[1]*Algames.state(pdtraj_1.pr[k])[px[i][2]]  for k=2:N] for i=1:p]
    u2 = [[vals[2]*Algames.state(pdtraj_2.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v2 = [[vals[2]*Algames.state(pdtraj_2.pr[k])[px[i][2]]  for k=2:N] for i=1:p]
    u3 = [[vals[3]*Algames.state(pdtraj_3.pr[k])[px[i][1]]  for k=2:N] for i=1:p]
    v3 = [[vals[3]*Algames.state(pdtraj_3.pr[k])[px[i][2]]  for k=2:N] for i=1:p]

    for i = 1:p
        quiver!(x[i],y[i],quiver=(β*u1[i],β*v1[i]), linewidth=1.5, color=:cyan)
        quiver!(x[i],y[i],quiver=(β*u2[i],β*v2[i]), linewidth=1.5, color=:red)
        # quiver!(x[i],y[i],quiver=(β*u3[i],β*v3[i]), linewidth=1.5, color=:black)
    end

    display(plt)
    return vals, pdtraj_ref, pdtraj_1, pdtraj_2
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
    # plot!(sqrt.(vals),
    plot!(vals,
        linetype=:steppost,
        linewidth=3.0,
        title="PCA on the Nash Equilibrium subspace \n around a nominal NE trajectory.",
        xlabel="eigval_index",
        ylabel="eigval_magnitude")
    display(plt)
	for (i,val) in enumerate(vals)
		println("($i,$val)")
	end
    return nothing
end





#
# ################################################################################
# # Toy Example
# ################################################################################
# using Plots
# T = Float64
#
# # Define the dynamics of the system
# p = 3 # Number of players
# # model = DoubleIntegratorGame(p=p) # game with 3 players with unicycle dynamics
# model = UnicycleGame(p=p) # game with 3 players with unicycle dynamics
# model = BicycleGame(p=p) # game with 3 players with unicycle dynamics
# n = model.n
# m = model.m
#
# # Define the horizon of the problem
# N = 20 # N time steps
# dt = 0.1 # each step lasts 0.1 second
# probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem
#
# # Define the objective of each player
# # We use a LQR cost
# Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p] # Quadratic state cost
# R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# # Desrired state
# xf = [SVector{model.ni[1],T}([2, 0.0,0,0]),
#       SVector{model.ni[2],T}([2, 0.0,0,0]),
#       SVector{model.ni[3],T}([2, 0.0,0,0]),
#       # SVector{model.ni[4],T}([3,+0.8,0,0]),
#       ]
# # Desired control
# uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
#
# # # Objectives of the game
# game_obj = GameObjective(Q,R,xf,uf,N,model)
# # radius = 0.5*ones(p)
# # μ = 4.0*ones(p)
# # μ = 4.0*[1.0, 0.1]
# # μ = 4.0*[0.1, 1.0]
# # add_collision_cost!(game_obj, radius, μ)
#
# # Define the constraints that each player must respect
# game_con = GameConstraintValues(probsize)
# # Add collision avoidance
# radius = 0.19
# add_collision_avoidance!(game_con, radius)
#
# # # Add control bounds
# # u_max =  5*ones(SVector{m,T})
# # u_min = -5*ones(SVector{m,T})
# # add_control_bound!(game_con, u_max, u_min)
#
# # # Add wall constraint
# # walls = [Wall([-1.0,-1.75], [1.0,0.25], [1.,-1.]/sqrt(2))]
# # add_wall_constraint!(game_con, walls)
#
# # # Add circle constraint
# # xc = [1., 2., 3.]
# # yc = [1., 2., 3.]
# # radius = [0.1, 0.2, 0.3]
# # add_circle_constraint!(game_con, xc, yc, radius)
#
# # Define the initial state of the system
# x0 = SVector{model.n,T}([
#     0.0, 0.0, 0.0, #0.0,
#    -0.4, 0.0, 0.4, #0.6,
#     0.0, 0.0, 0.0, #0.0,
#     0.0, 0.0, 0.0, #0.0,
#     ])
#
# # Define the Options of the solver
# opts = Options()
# opts.ls_iter = 15
# opts.outer_iter = 20
# opts.inner_iter = 20
# opts.ρ_0 = 1e0
# opts.reg_0 = 1e-5
# opts.α_dual = 1.0
# opts.λ_max = 1.0*1e7
# opts.ϵ_dyn = 1e-5
# opts.ϵ_sta = 1e-5
# opts.ϵ_con = 1e-5
# opts.ϵ_opt = 1e-5
# opts.regularize = true
# opts.αx_dual = ones(p)

# s = 0.5
# opts.αx_dual = [sqrt(s), 1/sqrt(s)]
# Define the game problem
# prob = GameProblem(N,dt,x0,model,opts,game_obj,game_con)
#
# # Solve the problem
# @time newton_solve!(prob)
# # @profiler newton_solve!(prob)
#
# plot!(prob.model, prob.pdtraj.pr)
# plot!(prob.stats)



using LinearAlgebra

A = [1. -1; -1 1]
isposdef(A)
