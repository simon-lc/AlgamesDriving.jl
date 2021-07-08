set_control!(z::Algames.KnotPoint, u) = z.z = [Algames.state(z); u]

function mpc_loop(prob::Algames.GameProblem, x0::SVx, sce::Scenario, vis::Vector{Visualizer},
    mpc_opts::MPCOptions{T}, mpc_stats::MPCStatistics{T}, traj_stats::TrajStatistics{T};
    seed::Vector{Int}=[1,6], display_traj=false, display_player=true) where {T,SVx}
    # We select the seed to make sure both players start with a different local nash equilibrium.
    probsize = prob.probsize
    p = probsize.p
    N = probsize.N
    pz = probsize.pz
    pu = probsize.pu

    # Initialization
    reset!(mpc_stats)
    reset!(traj_stats)
    probs = [deepcopy(prob) for i=1:p]

    prob.opts.shift = 2^10
    prob.opts.f_init = x -> (rand(x) .- 0.5)
    prob.opts.amplitude_init = 5.0

    # Set tolerances
    prob.opts.ϵ_opt = 3e-2
    prob.opts.ϵ_dyn = 3e-3
    prob.opts.ϵ_con = 3e-3
    prob.opts.ϵ_sta = 3e-3

    for i = 1:p
        probs[i].opts.inner_print = false
        probs[i].opts.outer_print = false
        probs[i].opts.inner_iter = 10
        probs[i].opts.outer_iter = 7
        probs[i].opts.seed = seed[1+(i-1)%p]
        probs[i].x0 = x0

        newton_solve!(probs[i])
        probs[i].opts.shift = 0
        probs[i].opts.amplitude_init = 1e-8
        plot!(probs[i].model, probs[i].pdtraj.pr)
    end


    dt = zeros(p)
    for j = 1:mpc_opts.M
        println("$(round(mpc_stats.t,digits=2))/$(round(mpc_opts.h, digits=1))s")
        trajs = deepcopy([probs[i].pdtraj.pr for i=1:p])
        # Solve
        for i = 1:p
            display_player ? set_state!(vis[i], probs[i].model, sce, probs[i].x0) : nothing
            display_traj ? set_line_traj!(vis[i], probs[i].model, sce.player, probs[i].pdtraj.pr) : nothing
            dt[i] = max(0.02, @elapsed newton_solve!(probs[i]))
        end
        # Record
        z0 = probs[1].pdtraj.pr[1]
        z = Algames.KnotPoint(Algames.state(z0), Algames.control(z0), maximum(dt))
        record!(traj_stats, z.dt, trajs)
        record!(mpc_stats, z)
        if mpc_stats.t > mpc_opts.h
            break
        end

        # Update solver
        X = []
        S = []
        for i = 1:p
            x,s = first_order_rollout(probs[i].pdtraj.pr, maximum(dt))
            push!(X, x)
            push!(S, s)
        end
        x = zeros(n)
        for i = 1:p
            x[pz[i]] = X[i][pz[i]]
        end
        x = SVector{n,T}(x)
        for i = 1:p
            probs[i].x0 = x
            probs[i].opts.shift = maximum(S)
        end
    end
    return nothing
end

function run_experiment(prob::Algames.GameProblem, x0::SVx, sce::Scenario, vis::Vector{Visualizer},
    mpc_opts::MPCOptions{T}; seed::Vector{Int}=[1,6], display_traj=false,
    display_player=false, n_sample::Int=5) where {T,SVx}
    mpc_stats = Vector{MPCStatistics}([])
    traj_stats = Vector{TrajStatistics}([])
    for l = 1:n_sample
        println("sample $l/$n_sample")
        m_stats = MPCStatistics()
        t_stats = TrajStatistics(probsize)
        xinit = random_initial_state(prob.model, x0, seed=l)
        mpc_loop(prob, xinit, sce, vis, mpc_opts, m_stats, t_stats,
            seed=[l,n_sample+l], display_traj=display_traj, display_player=display_player)
        push!(mpc_stats, m_stats)
        push!(traj_stats, t_stats)
    end
    return mpc_stats, traj_stats
end

function process_data(model::AbstractGameModel, mpc_stats::Vector{<:MPCStatistics},
    traj_stats::Vector{<:TrajStatistics}; upsampling::Int=3, α::T=1.0, atol::T=0.5) where {T}

    N = upsampling*Int(floor(mean([t_stats.iter for t_stats in traj_stats])))
    tf = α*mean([sum(t_stats.dt) for t_stats in traj_stats])
    t = zeros(N)
    X = []

    n_sample = length(mpc_stats)
    probsize = traj_stats[1].probsize
    p = probsize.p
    plt = plot()
    for l = 1:n_sample
        t_stats = traj_stats[l]
        for i = 1:p
            for j = i+1:p
                dist = [distance(model, trajs[i], trajs[j]) for trajs in t_stats.traj]
                plot!(cumsum(t_stats.dt), dist, linewidth=2.0, label="")
                scatter!(cumsum(t_stats.dt), dist, markersize=6.0, linewidth=2.0, label="$l",
                    markershapes=marker[1+(l-1)%length(marker)])
                x, t = resample(dist, cumsum(t_stats.dt), N, tf)
                push!(X,x)
            end
        end
    end

    x = mean(X)
    equa = [sum([X[l][i] .<= atol for l=1:n_sample]) for i=1:N]
    diff = [n_sample-equa[i] for i = 1:N]
    plot!(t, equa, linewidth=5.0, label="equa")
    plot!(t, diff, linewidth=5.0, label="diff")
    plot!(t, x, linewidth=5.0)
    display(plt)
    return x, t, equa, diff
end

function get_figure(x, t, equa, diff)
    N = length(t)
    plt = plot(
        xlim=(0,Inf),
        ylim=(0,Inf),
        xlabel="time (s)",
        ylabel="Nash equilibrium mismatch")

    plot!(t,zeros(N), label="Equilibrum match", ribbon=(zeros(N),ones(N)), color=:blue, fillalpha=0.7)
    plot!(t,zeros(N), label="Equilibrum mismatch", ribbon=(zeros(N),diff/(equa[1]+diff[1])), color=:orange, fillalpha=1.0)
    plot!(t,x./maximum(x),linewidth=3.0, label="Normalized mean discrepancy")
    for k = 1:N
        println("($(t[k]), $(diff[k]))")
    end
    for k = 1:N
        println("($(t[k]), $(equa[k]))")
    end
    display(plt)
    return nothing
end

function resample(x::AbstractVector, t::AbstractVector, N::Int, tf::T) where {T}
    t_ = zeros(N)
    x_ = zeros(N)
    for i = 1:N
        τ = (i-1)*tf/N
        t_[i] = τ
        it = findfirst(x -> x>=τ, t)
        it == nothing ? it = length(x) : nothing
        x_[i] = x[it]
    end
    return x_, t_
end

function distance(model::AbstractGameModel, t1::Algames.Traj, t2::Algames.Traj)
    p = model.p
    d = 0.0
    N = length(t1)
    @assert N == length(t2)
    for k = 1:N
        x1 = Algames.state(t1[k])
        x2 = Algames.state(t2[k])
        vs1 = standardize(model, x1)
        vs2 = standardize(model, x2)
        for i = 1:p
            Δ = [vs1[i].x - vs2[i].x, vs1[i].y - vs2[i].y, vs1[i].θ - vs2[i].θ, vs1[i].v - vs2[i].v]
            d += norm(Δ)
        end
    end
    return d
end

function random_initial_state(model::AbstractGameModel, x0::SVx; amplitude::T=1.0, seed::Int=rand(1:100)) where {SVx}
    Random.seed!(seed)
    p = model.p
    vs = standardize(model, x0)
    x = amplitude*[0.10, .65]
    y = amplitude*[0.02, 0.03]
    θ = amplitude*[0.10, 0.05]
    v = amplitude*[0.02, 0.02]
    for i = 1:p
        vs[i].x += x[i]*(rand()-0.5)
        vs[i].y += y[i]*(rand()-0.5)
        vs[i].θ += θ[i]*(rand()-0.5)
        vs[i].v += v[i]*(rand()-0.5)
    end

    xinit = specialize(model, vs)
    return xinit
end
