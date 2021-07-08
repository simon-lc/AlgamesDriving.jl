T = Float64
# Create players
p = 2
model = DoubleIntegratorGame(p=p)
model = UnicycleGame(p=p)
n = model.n
m = model.m


# Create Roadway
roadway_opts = MergingRoadwayOptions(lane_width=0.26, merging_angle=pi/12, merging_point=1.5)
roadway = build_roadway(roadway_opts)

# Create Players
players = Vector{Player{T}}(undef, p)
player_opts = HighwayPlayerOptions{model.ni[1],model.mi[1],T}()
players[1] = Player(model, roadway.lane[5])
players[2] = Player(model, roadway.lane[3])

# Create Scenario
sce = Scenario(model, roadway, players)



# Define the horizon of the problem
N = 20 # N time steps
dt = 0.12 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(1*[0.0, 0.5, 1.0, 0.5]) for i=1:p] # Quadratic state cost
R = [Diagonal(0.1*ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
# xf = [SVector{model.ni[1],T}([1.0,+0.10,0.50,0]),
#       SVector{model.ni[2],T}([1.0,-0.10,0.50,0]),
#       ]
xf = [SVector{model.ni[1],T}([1.0,-0.10,0.0,0.50]),
    SVector{model.ni[2],T}([1.0,-0.10,0.0,0.75]),
    ]

# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = Algames.GameObjective(Q,R,xf,uf,N,model)
radius = 0.25*ones(p)
μ = 1.0*ones(p)
add_collision_cost!(game_obj, radius, μ)

# Define the constraints that each player must respect
game_con = Algames.GameConstraintValues(probsize)
game_con = GameConstraintValues(N, sce)
u_max =  SVector{m,T}([1.5, 1.5, 0.75, 0.75])
u_min =  SVector{m,T}([-1.5, -1.5, -0.75, -0.75])
add_control_bound!(game_con, u_max, u_min)


# Define the initial state of the system
# x0 = SVector{model.n,T}([
#     0.25, 0.35,
#    -0.45, -0.08,
#     0.4,   0.55,
#     0.1,   0.0,
#     ])
x0 = SVector{model.n,T}([
    0.25, -0.30,
   -0.45, -0.08,
   pi/12,   0.0,
    0.50,   0.55,
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
        # plot!(probs[i].stats)
    end


    dt = zeros(p)
    for j = 1:mpc_opts.M
        println("$(round(mpc_stats.t,digits=2))/$(round(mpc_opts.h, digits=1))s")
        trajs = deepcopy([probs[i].pdtraj.pr for i=1:p])
        # Solve
        for i = 1:p
            display_player ? set_state!(vis[i], probs[i].model, sce, probs[i].x0) : nothing
            display_traj ? set_line_traj!(vis[i], probs[i].model, sce.player, probs[i].pdtraj.pr) : nothing
            # # set the controls of the other players to 0
            # if i == 1
            #     for k = 1:N-1
            #         z = probs[i].pdtraj.pr[k]
            #         u = Vector(Algames.control(z))
            #         for l = 1:p
            #             u[pu[i]] *= 0.0
            #         end
            #         set_control!(z, u)
            #     end
            # end
            # for k = 1:N-1
            #     z = probs[i].pdtraj.pr[k]
            #     u = Vector(Algames.control(z))
            #     for l ∈ setdiff(1:p, i)
            #         u[pu[i]] *= 0.0
            #     end
            #     set_control!(z, u)
            # end
            dt[i] = max(0.02, @elapsed newton_solve!(probs[i]))
        end
        # Record
        z0 = probs[1].pdtraj.pr[1]
        z = Algames.KnotPoint(Algames.state(z0), Algames.control(z0), maximum(dt))
        record!(traj_stats, z.dt, trajs)
        # @show traj_stats.iter
        # @show length(traj_stats.traj[1])

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
        # sleep(0.6)
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


# # Initialize visualizers
# vis = [Visualizer() for i=1:p]
# open.(vis)
#
# # Visualize trajectories
# for i = 1:p
#     set_scenario!(vis[i], sce)
#     set_env!(vis[i], VehicleState(1.0, 0.0, 0.0, 0.0))
#     set_camera_birdseye!(vis[i], height=11.0)
# end


mpc_opts = MPCOptions(M=40, h=2.5)
m_stats = MPCStatistics()
t_stats = TrajStatistics(probsize)

for i = 1:20
    xinit = random_initial_state(model, x0, amplitude=1., seed=i)
    set_state!(vis[1], model, sce, xinit)
    sleep(0.1)
end

for i = 1:5
    xinit = random_initial_state(model, x0, seed=i, amplitude=1.0)
    mpc_loop(prob, xinit, sce, vis, mpc_opts, m_stats, t_stats, seed=[3,10],
        display_traj=true, display_player=true)
end

# @time mpc_loop(prob, x0, sce, vis, mpc_opts, m_stats, t_stats, seed=[7,6])


mpc_opts = MPCOptions(M=500, h=4.0)
# mpc_stats, traj_stats = run_experiment(prob, x0, sce, vis, mpc_opts,
    # n_sample=100, display_traj=true, display_player=true)
x, t, e, d = process_data(model, mpc_stats, traj_stats, α=0.25, upsampling=1, atol=2.0)
get_figure(x,t,e,d)




mpc_stats_100 = deepcopy(mpc_stats)
traj_stats_100 = deepcopy(traj_stats)

marker = [:none, :auto, :circle, :rect, :star5,
    :diamond, :hexagon, :cross, :xcross, :utriangle,
    :dtriangle, :rtriangle, :ltriangle, :pentagon,
    :heptagon, :octagon, :star4, :star6, :star7, :star8, :vline, :hline, :x]
scatter([1,2,3], [4,5,3,], markershapes=marker[4])

t_stats = traj_stats[1]
for t = 1:length(t_stats.dt)
    for i = p:-1:1
        set_line_traj!(vis[i], model, sce.player, t_stats.traj[t][i])
        sleep(0.1)
    end
end
