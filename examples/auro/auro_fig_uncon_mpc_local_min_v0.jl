# Create Roadway
roadway_opts = MergingRoadwayOptions(lane_width=0.26)
roadway = build_roadway(roadway_opts)

 # Create Players
players = Vector{Player{T}}(undef, p)
player_opts = HighwayPlayerOptions{model.ni[1],model.mi[1],T}()
players[1] = Player(model, roadway.lane[5])
players[2] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)




# Create players
T = Float64
p = 2
model = DoubleIntegratorGame(p=p)
n = model.n
m = model.m

# Define the horizon of the problem
N = 20 # N time steps
dt = 0.1 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(3*[0.0, 1.0, 2.0, 1.0]) for i=1:p] # Quadratic state cost
R = [Diagonal(0.3*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([1.0,+0.35,0.5,0]),
      SVector{model.ni[2],T}([1.0,-0.05,0.5,0]),
      ]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.2*ones(p)
μ = 15.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
game_con = GameConstraintValues(N, sce)


# Define the initial state of the system
x0 = SVector{model.n,T}([
    0.2,   0.2,
   -0.45, -0.05,
    0.4,   0.5,
    0.1,   0.0,
    ])

# Define the Options of the solver
opts = Options()
opts.ls_iter = 15
opts.outer_iter = 20
opts.inner_iter = 20
opts.ρ_0 = 1e0
opts.reg_0 = 1e-8
opts.α_dual = 0.0
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


function mpc_loop(prob::Algames.GameProblem, x0::SVx, sce::Scenario, vis::Vector{Visualizer},
    mpc_opts::MPCOptions{T}, mpc_stats::MPCStatistics{T}, traj_stats::TrajStatistics{T};
    seed::Vector{Int}=[1,6]) where {T,SVx}
    # We select the seed to make sure both players start with a different local nash equilibrium.
    probsize = prob.probsize
    p = probsize.p
    pz = probsize.pz

    # Initialization
    reset!(mpc_stats)
    reset!(traj_stats)
    probs = [deepcopy(prob) for i=1:p]

    prob.opts.shift = 2^10
    prob.opts.f_init = x -> (rand(x) .- 0.5)
    prob.opts.amplitude_init = 3.0

    # Set tolerances
    prob.opts.ϵ_opt = 3e-3
    prob.opts.ϵ_dyn = 3e-3
    prob.opts.ϵ_con = 3e-3
    prob.opts.ϵ_sta = 3e-3
    prob.opts.inner_print = false
    prob.opts.outer_print = false

    for i = 1:p
        probs[i].opts.seed = seed[1+(i-1)%p]
        probs[i].x0 = x0

        newton_solve!(probs[i])
        probs[i].opts.shift = 0
        probs[i].opts.amplitude_init = 1e-8
        plot!(probs[i].model, probs[i].pdtraj.pr)
    end


    dt = zeros(p)
    for j = 1:mpc_opts.M
        println("$(round(mpc_stats.t,digits=2))/$(round(mpc_opts.h, digits=1))")
        trajs = deepcopy([probs[i].pdtraj.pr for i=1:p])
        # Solve
        for i = 1:p
            set_state!(vis[i], probs[i].model, sce, probs[i].x0)
            set_line_traj!(vis[i], probs[i].model, sce.player, probs[i].pdtraj.pr)
            dt[i] = @elapsed newton_solve!(probs[i])
        end
        # Record
        z0 = probs[1].pdtraj.pr[1]
        z = Algames.KnotPoint(Algames.state(z0), Algames.control(z0), minimum(dt))
        record!(traj_stats, z.dt, trajs)
        @show traj_stats.iter
        @show length(traj_stats.traj[1])

        record!(mpc_stats, z)
        if mpc_stats.t > mpc_opts.h
            break
        end

        # Update solver
        X = []
        S = []
        for i = 1:p
            x,s = first_order_rollout(probs[i].pdtraj.pr, minimum(dt))
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
            probs[i].opts.shift = minimum(S)
        end
    end
    return nothing
end

# # Initialize visualizers
# vis = [Visualizer() for i=1:p]
# open.(vis)

# # Visualize trajectories
# for i = 1:p
#     set_scenario!(vis[i], sce)
#     set_env!(vis[i], VehicleState(1.0, 0.0, 0.0, 0.0))
#     set_camera_birdseye!(vis[i], height=8.0)
# end

mpc_opts = MPCOptions(M=100, h=4.0)
mpc_stats = MPCStatistics()
traj_stats = TrajStatistics(probsize)

mpc_loop(prob, x0, sce, vis, mpc_opts, mpc_stats, traj_stats, seed=[5,13])

for t = 1:traj_stats.iter
    for i = 1:p
        set_line_traj!(vis[i], model, sce.player, traj_stats.traj[t][i])
        sleep(traj_stats.dt[t])
    end
end

# for i = 1:15
#     @show i
#     mpc_loop(prob, x0, sce, vis, mpc_opts, mpc_stats, seed=[i,i])
#     sleep(0.5)
# end

# init_traj, sol_traj = get_figure(prob, sce, vis_2, vis_1)
