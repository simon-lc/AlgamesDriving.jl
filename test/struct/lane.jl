@testset "Lane" begin

    # Test Lane
    id = 1
    walls = [Wall([0.1, 0.2], [0.3, 0.4], [0.5, 0.5]) for i=1:10]
    circles = [Circle(0.1, 0.2, 0.3) for i=1:10]
    start = StartingArea()
    f = x -> x
    lane = Lane(id, walls, circles, start, f)
    @test typeof(lane) <:Lane

end
