@testset "Roadway" begin

    # Test Roadway
    l = 5
    lanes = [Lane() for i=1:l]
    lanes[1].id = 1
    lanes[2].id = 2
    lanes[3].id = 3
    lanes[4].id = 4
    lanes[5].id = 5
    roadway = Roadway(lanes)
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    @test roadway.lane[4].id == 4
    @test roadway.lane[5].id == 5

    l = 5
    lanes = [Lane() for i=1:l]
    lanes[1].id = 5
    lanes[2].id = 3
    lanes[3].id = 2
    lanes[4].id = 1
    lanes[5].id = 4
    roadway = Roadway(lanes)
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    @test roadway.lane[4].id == 4
    @test roadway.lane[5].id == 5

    l = 5
    lanes = [Lane() for i=1:l]
    lanes[1].id = 1
    lanes[2].id = 1
    lanes[3].id = 2
    lanes[4].id = 2
    lanes[5].id = 10
    roadway = Roadway(lanes)
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    @test roadway.lane[4].id == 4
    @test roadway.lane[5].id == 5

    # Test add_lane!
    l = 5
    roadway = Roadway([Lane() for i=1:l])
    lane = Lane()
    lane.id = 10

    add_lane!(roadway, lane)
    @test roadway.l == 6
    @test roadway.lane[end].id == 6

end
