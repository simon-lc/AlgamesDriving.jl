@testset "MPC Struct" begin

    T = Float64
    N = 10
    p = 3
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N, model)

    mpc_opts = MPCOptions()
    m_stats = MPCStatistics{T}(12, 1.0, [2.0], Vector{Algames.KnotPoint}())
    @test m_stats.iter == 12
    @test m_stats.t == 1.0
    @test m_stats.dt == [2.0]
    @test m_stats.traj == Vector{Algames.KnotPoint}()

    reset!(m_stats)
    @test m_stats.iter == 0
    @test m_stats.t == 0.0
    @test m_stats.dt == zeros(0)
    @test m_stats.traj == Vector{Algames.KnotPoint}()

    x = rand(SVector{model.n,T})
    u = rand(SVector{model.m,T})
    dt = rand()
    z = Algames.KnotPoint(x,u,dt)
    record!(m_stats, z)

    @test m_stats.iter == 1
    @test m_stats.t == dt
    @test m_stats.dt == [dt]
    @test m_stats.traj[1] == z


    t_stats = TrajStatistics{T}(12, probsize, 1.0, [1.2], Vector{Vector{Algames.Traj}}())
    @test t_stats.iter == 12
    @test t_stats.probsize == probsize
    @test t_stats.t == 1.0
    @test t_stats.dt == [1.2]
    @test t_stats.traj == Vector{Vector{Algames.Traj}}()

    reset!(t_stats)
    @test t_stats.iter == 0
    @test t_stats.probsize == probsize
    @test t_stats.t == 0.0
    @test t_stats.dt == zeros(0)
    @test t_stats.traj == Vector{Vector{Algames.Traj}}()

    dt = rand()
    traj = Algames.Traj([z for k = 1:N])
    trajs = Vector{Algames.Traj}([traj for k = 1:p])
    record!(t_stats, dt, trajs)
    @test t_stats.iter == 1
    @test t_stats.t == dt
    @test t_stats.dt == [dt]
    @test t_stats.traj[1] == trajs


end
