@testset "MPC Methods" begin

    N = 10
    dt = 0.1
    p = 2
    model = UnicycleGame(p=p)
    opts = Options(inner_print=0.0, outer_print=0.0)
    roadway_opts = HighwayRoadwayOptions()
    roadway = build_roadway(roadway_opts)
    player = Vector{Player}(undef,p)
    player[1] = Player(model, roadway.lane[1])
    player[2] = Player(model, roadway.lane[2])
    sce = Scenario(model, roadway, player)
    vis = Visualizer()
    prob = GameProblem(N,dt,sce,opts)
    m_stats = MPCStatistics()
    m_opts = MPCOptions(M=5)
    x0 = get_state(sce)
    simulate_MPC!(vis, prob, x0, sce, m_stats, m_opts)


end
