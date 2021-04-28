@testset "Player" begin

    # Test Player
    T = Float64
    p = 3
    model = UnicycleGame(p=p)
    x0 = @SVector [1.0, 2.0, 3.0, 4.0]
    id = 2
    lane_id = 1
    Q = Diagonal(rand(SVector{model.ni[1],T}))
    R = Diagonal(rand(SVector{model.mi[1],T}))
    xf = rand(SVector{model.ni[1],T})
    uf = rand(SVector{model.mi[1],T})
    u_min = rand(SVector{model.mi[1],T})
    u_max = rand(SVector{model.mi[1],T})
    v_min = rand()
    v_max = rand()
    r_col = 1.0
    r_cost = 1.0
    μ = 1.0
    player = Player(x0, id, lane_id, Q, R, xf, uf, u_min, u_max, v_min, v_max, r_col, r_cost, μ)
    @test typeof(player) <: Player

    lane = Lane()
    player1 = Player(model, lane)
    @test typeof(player1) <: Player

end
