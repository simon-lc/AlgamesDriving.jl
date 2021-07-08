vis = Visualizer()
open(vis)

# Create Roadway
roadway_opts = HighwayRoadwayOptions()
roadway = build_roadway(roadway_opts)

# Create players
# T = Float64
p = 3
model = UnicycleGame(p=p)

players = Vector{Player}(undef, p)
players[1] = Player(model, roadway.lane[1],
    x0=VehicleState(0.0, 0.1, 0.0, 0.3),
    xf=VehicleState(4.0, 0.1, 0.0, 0.3))
players[2] = Player(model, roadway.lane[2],
    x0=VehicleState(0.0,-0.1, 0.0, 0.3),
    xf=VehicleState(4.0,-0.1, 0.0, 0.3))
players[3] = Player(model, roadway.lane[3],
    x0=VehicleState(0.2,-0.1, 0.0, 0.3),
    xf=VehicleState(4.0, 0.1, 0.0, 0.3))

# Create Scenario
sce = Scenario(model, roadway, players)

# Create AutoProblem
N = 20
dt = 0.1
solver_opts = Options()
solver_opts.inner_print = false
solver_opts.outer_print = false
prob = GameProblem(N, dt, sce, solver_opts)
@time newton_solve!(prob)

plot!(model, prob.pdtraj.pr)
plot!(prob.stats)

# Visualize trajectory
set_scenario!(vis, sce)
@time set_state!(vis, model, sce, Algames.state(prob.pdtraj.pr[10]))
@time set_traj!(vis, model, sce, prob.pdtraj.pr)




function simulate_game!(vis::Visualizer, prob::Algames.GameProblem, x0::SVx, sce::Scenario{T},
    player_id::Int, mpc_stats::MPCStatistics, mpc_opts::MPCOptions) where {T,SVx}
	N = prob.probsize.N
	m = prob.probsize.m
	pu = prob.probsize.pu
    # Initialization
    reset!(mpc_stats)
    prob.x0 = x0
    newton_solve!(prob)
    prob.opts.shift = 0
	no_input_ct = 0
	char = 'N'

    for j = 1:mpc_opts.M
		@show j
        # Solve
        set_state!(vis, prob.model, sce, prob.x0)

		c, empty_stream  = get_char()
		if empty_stream
			no_input_ct += 1
		else
			char = c
			no_input_ct = 0
		end
		if no_input_ct > 5
			char = 'N'
		end
		if char == 'N'
			ui = [ 0., 0.]
		elseif char == 'A'
			ui = [ 0.,  0.3]
		elseif char == 'B'
			ui = [ 0., -0.3]
		elseif char == 'D'
			ui = [ 0.3, 0.]
		elseif char == 'C'
			ui = [-0.3, 0.]
		end

		dt = @elapsed newton_solve!(prob)
        z0 = prob.pdtraj.pr[1]
        z = Algames.KnotPoint(Algames.state(z0), Algames.control(z0), dt)
        record!(mpc_stats, z)
        if mpc_stats.t > mpc_opts.h
            break
        end
        # Update solver
		# Insert player's control
		for k = 1:N-1
			zk = prob.pdtraj.pr[k]
			xk = Algames.state(zk)
			uk = Algames.control(zk)
			dtk = zk.dt
			uk = Vector(uk)
			uk[pu[player_id]] = ui
			uk = SVector{m,T}(uk)
			prob.pdtraj.pr[k] = Algames.KnotPoint(xk,uk,dtk)
		end
		Algames.rollout!(Algames.RK3, prob.model, prob.pdtraj.pr)

        x,s = first_order_rollout(prob.pdtraj.pr, dt)
        prob.x0 = x
        prob.opts.shift = s
    end
    return nothing
end


mpc_stats = MPCStatistics()
mpc_opts = MPCOptions(h=20.0, M=1000)
# simulate_MPC!(vis, prob, get_state(sce), sce, mpc_stats, mpc_opts)


simulate_game!(vis, prob, get_state(sce), sce, 1, mpc_stats, mpc_opts)



mpc_stats.traj

histogram(mpc_stats.dt)
mpc_stats.dt
mpc_stats.t
@time set_traj!(vis, model, sce, mpc_stats.traj)
