function display_arrow(prob::Algames.GameProblem, sce::Scenario, ref_pdtraj::PrimalDualTraj, val::T,
	eig_pdtraj::PrimalDualTraj; vis_opts::PlayerVisualizationOptions=PlayerVisualizationOptions{T}(),
	color=:orange, key::Int=0, β::T=1.2*10^5, height::T=0.01) where {T}

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
				shaft_radius=0.008,
				max_head_radius=0.020)

			t = Translation(vs_ref[i].x, vs_ref[i].y, height)
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

        opt = mean(abs.(prob_.stats.opt_vio[end].vio))
        sta = mean(abs.(prob_.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob_.stats.dyn_vio[end].vio))
        con = mean(abs.(prob_.stats.con_vio[end].vio))
        opts = prob_.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            val += 1
			@show val
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



# Newton Solver
function simple_projection(α::T, λ::AbstractVector, x::AbstractVector, v::AbstractVector,
	β::T, ascore::ActiveSetCore, prob::Algames.GameProblem, vmask::Vector{Int}) where {T}
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)
	nx = Sh
	nλ = Sv
	ix = Vector(1:nx)
	iλ = Vector(nx .+ (1:nλ))
	jmask = vcat(ix[vmask], iλ[vmask])

	xp = deepcopy(x + α*v)

	for l = 1:1
		set_primal_dual!(ascore, prob, xp)
		#################
		prob.opts.shift = 0
		prob.opts.α_dual = 0.0
		prob.opts.αx_dual = 0.05*α*ones(p)
		prob.opts.dual_reset = false
		prob.opts.ρ_0 = prob.pen.ρ[1]
		prob.opts.ρ_max = prob.pen.ρ[1]
		prob.opts.inner_print = false
		prob.opts.outer_print = false
		prob.opts.inner_iter = 20
		prob.opts.outer_iter = 60
		prob.opts.reg_0 = 1e-6
		ϵ = 1e-6
		prob.opts.ϵ_opt = ϵ
		prob.opts.ϵ_dyn = ϵ
		prob.opts.ϵ_con = ϵ
		prob.opts.ϵ_sta = ϵ
		newton_solve!(prob)
		xp = get_primal_dual(ascore, prob)
		println("f_no_change", scn(f(x, α, x, v, β)))
		println("f_solution ", scn(f(xp, α, x, v, β)))
		#################
		record!(prob.stats, prob.core, prob.model,
			prob.game_con, prob.pdtraj, prob.opts.outer_iter)
		opt = mean(abs.(prob.stats.opt_vio[end].vio))
		sta = mean(abs.(prob.stats.sta_vio[end].vio))
		dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
		con = mean(abs.(prob.stats.con_vio[end].vio))
		println("opt = "*scn(opt))
		println("sta = "*scn(sta))
		println("dyn = "*scn(dyn))
		println("con = "*scn(con))
	end
	return xp, λ
end


# Utils
function get_primal_dual(ascore::ActiveSetCore, prob::Algames.GameProblem)
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)

	y = zeros(S)
	x = zeros(Sh)
	cs = CStamp()
	get_traj!(prob.core, prob.pdtraj, y)
	x[1:S] = y
	for k = 1:N-1
		for i = 1:p
			for j = 1:p
				conval, v = get_collision_conval(prob.game_con, i,j)
				if v
					for (l,k) in enumerate(conval.inds)
						stampify!(cs, :h, :col, i, j, k)
						x[horizontal_idx(ascore, cs)] = conval.λ[l]
					end
				end
			end
		end
	end
	return x
end

function set_primal_dual!(ascore::ActiveSetCore, prob::Algames.GameProblem, x::AbstractVector)
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S

	set_traj!(prob.core, prob.pdtraj, x[1:S])

	cs = CStamp()
	for k = 1:N-1
		for i = 1:p
			for j = 1:p
				conval, v = get_collision_conval(prob.game_con, i,j)
				if v
					for (l,k) in enumerate(conval.inds)
						stampify!(cs, :h, :col, i, j, k)
						conval.λ[l] = x[horizontal_idx(ascore, cs)]
					end
				end
			end
		end
	end
	return nothing
end

function f(xp::AbstractVector, α::T, x::AbstractVector, v::AbstractVector, β::T) where {T}
	Δ = (xp - (x + α*v))
	return 0.5*β*Δ'*Δ
end


function Algames.scn(a::Number; digits::Int=1)
	@assert digits >= 0
    # a = m x 10^e
    if a == 0
        e = 0
        m = 0.0
    else
        e = Int(floor(log(abs(a))/log(10)))
        m = a*exp(-e*log(10))
    end
    m = round(m, digits=digits)
    if digits == 0
        m = Int(floor(m))
		strm = string(m)
	else
		strm = string(m)
		is_neg = m < 0.
		strm = strm*"0"^abs(2+digits+is_neg-length(strm))
    end
    sgn = a >= 0 ? " " : ""
    sgne = e >= 0 ? "+" : ""
    return "$sgn$(strm)e$sgne$e"
end
