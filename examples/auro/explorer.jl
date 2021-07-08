
# Evaluation
function f(xp::AbstractVector, α::T, x::AbstractVector, v::AbstractVector, β::T) where {T}
	Δ = (xp - (x + α*v))
	return 0.5*β*Δ'*Δ
end

function c(x::AbstractVector, ascore::ActiveSetCore, prob::Algames.GameProblem)
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)

	set_primal_dual!(ascore, prob, x)
	residual!(ascore, prob, prob.pdtraj)
	out = deepcopy(ascore.res)
	return out
end

function L(xp::AbstractVector, α::T, λ::AbstractVector, x::AbstractVector,
	v::AbstractVector, β::T, ascore::ActiveSetCore, prob::Algames.GameProblem, vmask::Vector{Int}) where {T}

	out = f(xp, α, x, v, β) + λ[vmask]'*c(xp, ascore, prob)[vmask]
	return out
end

# Gradients
function Algames.residual_jacobian!(ascore::ActiveSetCore, prob::Algames.GameProblem{KN,n,m,T,SVd,SVx},
	pdtraj::PrimalDualTraj{KN,n,m,T,SVd}) where {KN,n,m,T,SVd,SVx}

	probsize = ascore.probsize
	N = probsize.N
	p = probsize.p
	S = probsize.S

	# Reset!
	sparse_zero!(ascore.jac)

	residual_jacobian!(prob, pdtraj)
	# regularize_residual_jacobian!(prob) #############################
	ascore.jac[1:S,1:S] .= prob.core.jac

    # Allocations
	vs = VStamp()
	hs = HStamp()
    cs = CStamp()

	jacobian!(prob.game_con, pdtraj.pr)
	for i = 1:p
		for j ∈ setdiff(1:p,i)
			conval, v = get_collision_conval(prob.game_con, i, j)
			if v
				for (l,k) in enumerate(conval.inds)
					stampify!(vs, :opt, i, :x, 1, k)
					stampify!(cs, :h, :col, i, j, k)
					if valid(cs,N,p) && valid(vs,N,p)
						add2sub(ascore.jac_sub[(vs,cs)], conval.jac[l]')
					end
					if valid(hs,N,p) && valid(cs,N,p)
						stampify!(hs, :x, 1, k)
						stampify!(cs, :v, :col, i, j, k)
						add2sub(ascore.jac_sub[(hs,cs)], conval.jac[l])
					end
				end
			end
		end
	end
    return nothing
end

function ∇c(x::AbstractVector,ascore::ActiveSetCore, prob::Algames.GameProblem) where {T}
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)

	∇ = zeros(Sv,Sh)
	set_primal_dual!(ascore, prob, x)
	residual_jacobian!(ascore, prob, prob.pdtraj)
	@assert all(size(ascore.jac) .== size(∇))
	∇ = ascore.jac
	return ∇
end


function grad(xp::AbstractVector, α::T, λ::AbstractVector, x::AbstractVector,
	v::AbstractVector, β::T, ascore::ActiveSetCore, prob::Algames.GameProblem, vmask::Vector{Int}) where {T}
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)
	nx = Sh
	# nα = 1
	nλ = Sv
	ix = Vector(1:nx)
	# iα = Vector(nx .+ (1:nα))
	iλ = Vector(nx .+ (1:nλ))
	# iλ = Vector(nx + nα .+ (1:nλ))
	jmask = vcat(ix[vmask], iλ[vmask])

	∇ = ∇c(xp, ascore, prob)
	gx = β * (xp - (x+α*v)) + ∇'*λ
	# gx = (-β/length(x)^2 * (xp - (x+α*v)) + ∇'*λ)[vmask]
	# gα = [-β/length(x)^2 * (xp - (x+α*v))'*v - 1.0]
	gλ = c(xp, ascore, prob)
	g = vcat(gx, gλ)[jmask]
	# g = vcat(gx, gα, gλ)
	return g
end

# Jacobians

function jac(xp::AbstractVector, α::T, λ::AbstractVector, x::AbstractVector,
	v::AbstractVector, β::T, ascore::ActiveSetCore, prob::Algames.GameProblem, vmask::Vector{Int}) where {T}
	probsize = prob.probsize
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = probsize.S
	Sh = S + p*(p-1)*(N-1)
	Sv = S + Int(p*(p-1)*(N-1)/2)
	nx = Sh
	# nα = 1
	nλ = Sv
	ix = Vector(1:nx)
	# iα = Vector(nx .+ (1:nα))
	iλ = Vector(nx .+ (1:nλ))
	# iλ = Vector(nx + nα .+ (1:nλ))
	jmask = vcat(ix[vmask], iλ[vmask])
	# jmask = vcat(ix[vmask], iα, iλ[vmask])

	∇ = ∇c(xp, ascore, prob)
	j = zeros(nx+nλ, nx+nλ)
	# j = zeros(nx+nα+nλ, nx+nα+nλ)
	j[ix,ix] += β*I
	# j[ix,ix] += β/length(x)^2*I
	# j[iα,ix] = -β/length(x)^2*v'
	# j[ix,iα] = -β/length(x)^2*v
	j[iλ,ix] = ∇
	j[ix,iλ] = ∇'
	j[iλ,iλ] -= 1e-3*I

	return j[jmask, jmask]
end



# Newton Solver
function newton_solver(α::T, λ::AbstractVector, x::AbstractVector, v::AbstractVector,
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
	# nα = 1
	nλ = Sv
	ix = Vector(1:nx)
	# iα = Vector(nx .+ (1:nα))
	iλ = Vector(nx .+ (1:nλ))
	# iλ = Vector(nx + nα .+ (1:nλ))
	jmask = vcat(ix[vmask], iλ[vmask])
	# jmask = vcat(ix[vmask], iα, iλ[vmask])

	xp = deepcopy(x + α*v)

	for l = 1:1
		# plot!(prob.model, prob.pdtraj.pr)
		@show scn(mean(abs.(c(xp, ascore, prob_copy2)[vmask])))

		g = grad(xp, α, λ, x, v, β, ascore, prob, vmask)
		println("***********L = ", scn(L(xp, α, λ, x, v, β, ascore, prob, vmask)))
		println("***********res = ",scn(mean(abs.(g))))
		j = jac(xp, α, λ, x, v, β, ascore, prob, vmask) + 1e-18*I
		Δm = - \(lu(j), g)
		println("Δm = ", scn(mean(abs.(Δm))))

		Δ = zeros(nx+nλ)
		# Δ = zeros(nx+nα+nλ)
		# Δ[jmask] = 0.01*Δm
		Δ[jmask] = 0.1*α*Δm/mean(abs.(Δm[ix[vmask]]))
		# Δ[jmask] = 0.05*α*Δm/mean(abs.(Δm[ix[vmask]]))
		println("Δ = ", scn(mean(abs.(Δ[jmask]))))
		println("Δx = ", scn(mean(abs.(Δ[ix[vmask]]))))
		println("α = ", scn(α))
		xp[vmask] += Δ[ix[vmask]]
		# α += Δ[iα][1]
		λ[vmask] += Δ[iλ[vmask]]
		set_primal_dual!(ascore, prob, xp)
		#################
		prob.opts.shift = 0
		prob.opts.α_dual = 0.0
		prob.opts.αx_dual = 0.0*1e-3*ones(p)
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






function finite_difference_grad(xp, α, λ, x, v, β, ascore, prob, vmask)
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

	function Lλ(λ)
		return L(xp, α, λ, x, v, β, ascore, prob, vmask)
	end
	function Lx(xp)
		return L(xp, α, λ, x, v, β, ascore, prob, vmask)
	end

	gx = finite_difference_gradient(Lx, xp)
	gλ = finite_difference_gradient(Lλ, λ)
	g = vcat(gx, gλ)
	return g[jmask]
end



function finite_difference_gradient(f, x; eps::T=1e-5)
	# the gradient dimension will be m = length(x)
	m = length(x)
	x = SVector{m,T}(x)
	grad = zeros(m)
	for k = 1:m
		epsk = zeros(m)
		epsk[k] = eps
		grad[k] = (f(x .+ epsk) - f(x .- epsk))/(2eps)
	end
	return grad
end

function finite_difference_hessian(f, x; eps::T=1e-5)
	# the Hessian dimension will be m.m = length(x).length(x)
	m = length(x)
	x = SVector{m,T}(x)
	hess = zeros(m,m)
	for k = 1:m
		for l = 1:m
			epsk = zeros(m)
			epsl = zeros(m)
			epsk[k] = eps
			epsl[l] = eps
			hess[k,l] = (f(x .+ epsk .+ epsl) - f(x .+ epsk .- epsl) - f(x .- epsk .+ epsl) + f(x .- epsk .- epsl))/(4eps^2)
		end
	end
	return hess
end

function finite_difference_jacobian(f, x; n::Int=length(f(x)), eps::T=1e-5)
	# n is the dimension of the output of f
	# the Jacobian dimension will be n.m = n.length(x)
	m = length(x)
	x = SVector{m,T}(x)
	jac = zeros(n,m)
	for k = 1:m
		epsk = zeros(m)
		epsk[k] = eps
		jac[:,k] = (f(x .+ epsk) - f(x .- epsk))/(2eps)
	end
	return jac
end
