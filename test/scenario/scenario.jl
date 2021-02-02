@testset "Scenario" begin

    # Test Scenario
    T = Float64
    N = 10
    p = 3
    model = UnicycleGame(p=p)
    roadway_opts = HighwayRoadwayOptions()
    roadway = build_roadway(roadway_opts)
    player = Vector{Player}(undef, p)
    player[1] = Player(model, roadway.lane[1])
    player[2] = Player(model, roadway.lane[2])
    player[3] = Player(model, roadway.lane[3])
    sce = Scenario(model, roadway, player)
    @test typeof(sce) <: Scenario

    x0 = zeros(model.n)
    x0[4] = roadway_opts.lane_width/2
    x0[5] = -roadway_opts.lane_width/2
    x0[6] = -roadway_opts.lane_width/2
    @test get_state(sce) == SVector{model.n}(x0)

    @test all(get_control_bounds(sce) .==
        (-Inf*ones(SVector{model.m,T}), Inf*ones(SVector{model.m,T})))

    game_obj = GameObjective(N, sce)
    @test typeof(game_obj) <: Algames.GameObjective

    game_con = GameConstraintValues(N, sce)
    @test typeof(game_con) <: Algames.GameConstraintValues

    dt = 0.1
    opts = Options()
    prob = GameProblem(N, dt, sce, opts)
    @test typeof(prob) <: Algames.GameProblem

end
