@testset "Lane" begin

    # Test circle
    x = 1.0
    y = 2.0
    r = 10.0
    c = CircularWall(x,y,r)
    @test typeof(c) <: CircularWall

    c1 = CircularWall()
    @test c1.x == 0.0
    @test c1.y == 0.0
    @test c1.r == 0.0

    # Test Lane
    id = 1
    walls = [Algames.Wall([0.1, 0.2], [0.3, 0.4], [0.5, 0.5]) for i=1:10]
    circles = [CircularWall(0.1, 0.2, 0.3) for i=1:10]
    start = StartingArea()
    f = x -> x
    lane = Lane(id, :lane_1, walls, circles, start, f)
    @test typeof(lane) <:Lane

end
