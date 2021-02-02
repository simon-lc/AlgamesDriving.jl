function first_order_rollout(traj::Algames.Traj, dt::T) where {T}
    N = length(traj)
    h = sum([traj[k].dt for k=1:N])
    @assert dt < h

    s = 0
    x = Algames.state(traj[1])
    while dt > 0
        z = traj[s+1]
        δ = z.dt
        if δ > dt
            x_1 = Algames.state(z)
            x1  = Algames.state(traj[s+2])
            x = x_1 + dt/δ*(x1 - x_1)
            break
        else
            dt -= δ
            s += 1
        end
    end
    return x, s
end

function simulate_MPC!(vis::Visualizer, prob::Algames.GameProblem, x0::SVx, sce::Scenario,
    mpc_stats::MPCStatistics, mpc_opts::MPCOptions) where {SVx}
    # Initialization
    reset!(mpc_stats)
    prob.x0 = x0
    newton_solve!(prob)
    prob.opts.shift = 0

    for j = 1:mpc_opts.M
        # Solve
        set_state!(vis, prob.model, sce, prob.x0)
        dt = @elapsed newton_solve!(prob)
        z0 = prob.pdtraj.pr[1]
        z = Algames.KnotPoint(Algames.state(z0), Algames.control(z0), dt)
        record!(mpc_stats, z)
        if mpc_stats.t > mpc_opts.h
            break
        end
        # Update solver
        x,s = first_order_rollout(prob.pdtraj.pr, dt)
        prob.x0 = x
        prob.opts.shift = s
    end
    return nothing
end
