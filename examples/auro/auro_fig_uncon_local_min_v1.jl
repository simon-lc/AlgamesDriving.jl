vis = Visualizer()
open(vis)

# Create Roadway
roadway_opts = MergingRoadwayOptions(lane_width=0.26)
roadway = build_roadway(roadway_opts)

# Create players
T = Float64
p = 2
model = DoubleIntegratorGame(p=p)

players = Vector{Player{T}}(undef, p)
player_opts = HighwayPlayerOptions{model.ni[1],model.mi[1],T}(r_cost=0.22)

players[1] = Player(model, roadway.lane[3],
    Q=Diagonal(SVector{model.ni[1],T}(0.0, 0.4, 0.4, 0.05)),
    R=Diagonal(SVector{model.mi[1],T}(0.1, 0.1)),
    x0=VehicleState(0.15, -0.1, 0.0, 0.7),
    xf=VehicleState(1.9, -0.1, 0.0, 0.7),
    μ=200.0,
    r_col=0.00,
    r_cost=0.16,
    opts=player_opts)
players[2] = Player(model, roadway.lane[6],
    Q=Diagonal(SVector{model.ni[2],T}(0.0, 0.4, 0.4, 0.05)),
    R=Diagonal(SVector{model.mi[2],T}(0.1, 0.1)),
    x0=roadway.lane[5].centerline(0.15, 0.7),
    xf=roadway.lane[5].centerline(1.9, 0.7),
    μ=200.0,
    r_col=0.00,
    r_cost=0.16,
    opts=player_opts)

# Create Scenario
sce = Scenario(model, roadway, players)

# Create AutoProblem
N = 50
dt = 0.05
solver_opts = Options()
solver_opts.inner_print = true
solver_opts.outer_print = true
solver_opts.reg_0 = 1e-5
prob = GameProblem(N, dt, sce, solver_opts)

init_traj!(prob.pdtraj, f=rand, amplitude=1e-8)
solver_opts.shift = 0
@time newton_solve!(prob)

plot!(model, prob.pdtraj.pr)
plot!(prob.stats)

# Visualize trajectory
set_scenario!(vis, sce)
set_env!(vis, VehicleState(2.0, 0.0, 0.0, 0.0))
set_camera_birdseye!(vis, height=7.3)
build_waypoint!(vis, sce.player, N)

@time set_state!(vis, model, sce, Algames.state(prob.pdtraj.pr[10]))
@time set_traj!(vis, model, sce, prob.pdtraj.pr)
@time set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr)

function vizz(prob)
    init_traj = Vector{PrimalDualTraj}()
    sol_traj = Vector{PrimalDualTraj}()

    prob.opts.shift = 2^10
    prob.opts.f_init = rand
    prob.opts.amplitude_init = 1e-1
    solver_opts.reg_0 = 1e-4
    for k = 1:10
        prob.opts.seed = k
        prob.opts.ϵ_opt = 1e-6
        prob.opts.ϵ_dyn = 1e-6
        prob.opts.ϵ_con = 1e-6
        prob.opts.ϵ_sta = 1e-6

        prob.opts.outer_iter = 0
        @time newton_solve!(prob)
        init = deepcopy(prob.pdtraj)
        #
        prob.opts.outer_iter = 20
        @time newton_solve!(prob)
        sol = deepcopy(prob.pdtraj)

        opt = mean(abs.(prob.stats.opt_vio[end].vio))
        sta = mean(abs.(prob.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
        con = mean(abs.(prob.stats.con_vio[end].vio))
        opts = prob.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            # plot!(prob.stats)
            # plot!(prob.model, init.pr)
            plot!(prob.model, sol.pr)
            @time set_waypoint_traj!(vis, model, sce, init.pr)
            # sleep(1.0)
            # plot!(prob.model, prob.pdtraj.pr)
            @time set_traj!(vis, model, sce, prob.pdtraj.pr)
            build_waypoint!(vis, sce.player, N, key=k)
            @time set_waypoint_traj!(vis, model, sce, sol.pr, key=k)
            sleep(4.0)
            # push!(init_traj, init)
            push!(sol_traj, sol)
        end
    end
    return init_traj, sol_traj
end

# init_traj, sol_traj = vizz(prob)
# init_traj
# sol_traj


init_traj!(prob.pdtraj; x0=prob.x0, f=prob.opts.f_init, amplitude=prob.opts.amplitude_init, s=prob.opts.shift)
Algames.rollout!(Algames.RK3, prob.model, prob.pdtraj.pr)
set_waypoint_traj!(vis, model, sce, prob.pdtraj.pr)
plot!(model, prob.pdtraj.pr)

function figgg(prob::Algames.GameProblem)
    prob.opts.shift = 2^10
    prob.opts.α_dual = 0.0
    prob.opts.αx_dual = zeros(p)
    prob.opts.f_init = x -> (rand(x) .- 0.5)
    prob.opts.amplitude_init = 1.0

    prob.opts.ϵ_opt = 1e-6
    prob.opts.ϵ_dyn = 1e-6
    prob.opts.ϵ_con = 1e-6
    prob.opts.ϵ_sta = 1e-6



    init = []
    sol = []
    plt = plot()
    plt2 = plot()
    for k = 1:10
        prob.opts.seed = k
        prob.opts.outer_iter = 0
        @time newton_solve!(prob)
        init_traj = deepcopy(prob.pdtraj)

        prob.opts.outer_iter = 20
        @time newton_solve!(prob)
        sol_traj = deepcopy(prob.pdtraj)
        plot!(prob.stats)

        opt = mean(abs.(prob.stats.opt_vio[end].vio))
        sta = mean(abs.(prob.stats.sta_vio[end].vio))
        dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
        con = mean(abs.(prob.stats.con_vio[end].vio))
        opts = prob.opts
        if opt < opts.ϵ_opt && sta < opts.ϵ_sta && dyn < opts.ϵ_dyn && con < opts.ϵ_con
            plot!(model, init_traj.pr, plt=plt)
            plot!(model, sol_traj.pr, plt=plt2)
            push!(sol, sol_traj)
            push!(init, init_traj)
        end
    end
    display(plt)
    return nothing
end

figgg(prob)

plot!
