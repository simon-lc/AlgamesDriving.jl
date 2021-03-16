@testset "Roadway" begin

    function test_centerline_continuity(f; v::T=1.0, s_min::T=-10.0, s_max::T=10.0,
        sample::Int=2000) where T
        out = true
        step = (s_max - s_min)/sample
        vs_ = f(s_min,v)
        vs = f(s_min,v)
        for k = 1:sample
            s = s_min + k*step
            vs = f(s,v)
            out &= norm([vs.x, vs.y] - [vs_.x, vs_.y]) <= (step + 1e-5)
            vs_ = vs
        end
        return out
    end

    # Test Roadway
    T = Float64
    opts = HighwayRoadwayOptions()
    l = 5
    lanes = [Lane() for i=1:l]
    lanes[1].id = 1
    lanes[2].id = 2
    lanes[3].id = 3
    lanes[4].id = 4
    lanes[5].id = 5
    roadway = Roadway(lanes, opts)
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
    roadway = Roadway(lanes, opts)
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
    roadway = Roadway(lanes, opts)
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    @test roadway.lane[4].id == 4
    @test roadway.lane[5].id == 5

    # Test add_lane!
    l = 5
    roadway = Roadway([Lane() for i=1:l], opts)
    lane = Lane()
    lane.id = 10

    add_lane!(roadway, lane)
    @test roadway.l == 6
    @test roadway.lane[end].id == 6

    # Test build_roadway
    opts = HighwayRoadwayOptions()
    roadway = build_roadway(opts)
    @test roadway.l == 3
    @test length(roadway.lane) == 3
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    f_left = roadway.lane[1].centerline
    f_right = roadway.lane[2].centerline
    f_both = roadway.lane[3].centerline
    @test test_centerline_continuity(f_left)
    @test test_centerline_continuity(f_right)
    @test test_centerline_continuity(f_both)
    @test typeof(f_left(0.0, 1.0)) <: VehicleState{T}

    s = rand()
    v = rand()
    @test f_left(s,v) == VehicleState(s,opts.lane_width/2,0.0,v)

    s = rand()
    v = rand()
    @test f_right(s,v) == VehicleState(s,-opts.lane_width/2,0.0,v)

    s = rand()
    v = rand()
    @test f_both(s,v) == VehicleState(s,-opts.lane_width/2,0.0,v)


    opts = MergingRoadwayOptions()
    roadway = build_roadway(opts)
    @test roadway.l == 6
    @test length(roadway.lane) == 6
    @test roadway.lane[1].id == 1
    @test roadway.lane[2].id == 2
    @test roadway.lane[3].id == 3
    f_straight_left = roadway.lane[1].centerline
    f_straight_right = roadway.lane[2].centerline
    f_straight_both = roadway.lane[3].centerline
    f_merging_left = roadway.lane[4].centerline
    f_merging_right = roadway.lane[5].centerline
    f_merging_both = roadway.lane[6].centerline
    @test typeof(f_straight_left(0.0, 1.0)) <: VehicleState{T}

    s = rand()
    v = rand()
    @test f_straight_left(s,v) == VehicleState(s,opts.lane_width/2,0.0,v)

    s = rand()
    v = rand()
    @test f_straight_right(s,v) == VehicleState(s,-opts.lane_width/2,0.0,v)

    s = rand()
    v = rand()
    @test f_straight_both(s,v) == VehicleState(s,-opts.lane_width/2,0.0,v)

    @test test_centerline_continuity(f_straight_left)
    @test test_centerline_continuity(f_straight_right)
    @test test_centerline_continuity(f_straight_both)
    @test test_centerline_continuity(f_merging_left)
    @test test_centerline_continuity(f_merging_right)
    @test test_centerline_continuity(f_merging_both)

    # FourIntersectionRoadway
    opts = FourIntersectionRoadwayOptions()
    roadway = build_roadway(opts)
    for i in length(roadway.lane)
        @test roadway.lane[i].id == i
        @test test_centerline_continuity(roadway.lane[i].centerline)
    end
end
