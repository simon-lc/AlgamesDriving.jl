@testset "Camera" begin

    T = Float64
    p = 3
    model = BicycleGame(p=p)

    roadway_opts = MergingRoadwayOptions(merging_point=1.4, lane_width=0.28)
    roadway = build_roadway(roadway_opts)

    players = Vector{Player{T}}(undef, p)
    players[1] = Player(model, roadway.lane[3])
    players[2] = Player(model, roadway.lane[3])
    players[3] = Player(model, roadway.lane[3])

    sce = Scenario(model, roadway, players)

    # Initialize visualizers
    vis = Visualizer()

    # Visualize trajectories
    set_scenario!(vis, sce)
    set_env!(vis, VehicleState(1.3, 0.0, 0.0, 0.0))
    set_camera!(vis)
    set_camera_birdseye!(vis, height=8.0)

end
