@testset "Starting Area" begin

    # Test Starting Area
    x_nom = VehicleState(1.0, 1.0, 1.0, 1.0)
    x_min = VehicleState(0.0, 0.0, 0.0, 0.0)
    x_max = VehicleState(1.0, 2.0, 3.0, 4.0)
    start1 = StartingArea(x_nom, x_min, x_max)
    start2 = StartingArea(x_nom)
    start3 = StartingArea(x_min, x_max)

    @test start1.x_nom == x_nom
    @test start1.x_min == x_min
    @test start1.x_max == x_max

    @test start2.x_nom == x_nom
    @test start2.x_min == x_nom
    @test start2.x_max == x_nom

    @test start3.x_nom == VehicleState(0.5, 1.0, 1.5, 2.0)
    @test start3.x_min == x_min
    @test start3.x_max == x_max

    xr = randstate(start1)
    @test 0.0 <= xr.x <= 1.0
    @test 0.0 <= xr.y <= 2.0
    @test 0.0 <= xr.θ <= 3.0
    @test 0.0 <= xr.v <= 4.0

end
