@testset "VehicleState" begin

    # Test Base
    x = VehicleState()
    xc = copy(x)
    @test x == xc
    xc.x = 10
    @test x != xc

    x1 = VehicleState(1.0,2.0,3.0,4.0)
    x2 = VehicleState(2.0,3.0,4.0,5.0)
    x3 = x1 + x2
    @test x3.x == 3.0
    @test x3.y == 5.0
    @test x3.θ == 7.0
    @test x3.v == 9.0

    x4 = x2*0.5
    @test x4.x == 1.0
    @test x4.y == 1.5
    @test x4.θ == 2.0
    @test x4.v == 2.5

    x4 = 0.5*x2
    @test x4.x == 1.0
    @test x4.y == 1.5
    @test x4.θ == 2.0
    @test x4.v == 2.5

    # Test standardize
    # DoubleIntegrator
    model = Algames.DoubleIntegratorGame(p=3)
    xi = @SVector [1.0, 2.0, sqrt(3)/2, 0.5]
    vsi = standardize(model, xi)

    @test vsi.x == xi[1]
    @test vsi.y == xi[2]
    @test abs(vsi.θ - pi/6) < 1e-10
    @test abs(vsi.v - 1) < 1e-10

    x = @SVector [1.0, 1.2, 1.4,
                  2.0, 2.2, 2.4,
                  sqrt(3)/2, sqrt(3)/2, sqrt(3)/2,
                  0.5, 0.5, 0.5]
    vs = standardize(model, x)

    @test vs[1].x == 1.0
    @test vs[1].y == 2.0
    @test abs(vs[1].θ - pi/6) < 1e-10
    @test abs(vs[1].v - 1) < 1e-10

    @test vs[2].x == 1.2
    @test vs[2].y == 2.2
    @test abs(vs[2].θ - pi/6) < 1e-10
    @test abs(vs[2].v - 1) < 1e-10

    @test vs[3].x == 1.4
    @test vs[3].y == 2.4
    @test abs(vs[3].θ - pi/6) < 1e-10
    @test abs(vs[3].v - 1) < 1e-10

    # Unicycle
    model = UnicycleGame(p=3)
    xi = @SVector [1.0, 2.0, sqrt(3)/2, 0.5]
    vsi = standardize(model, xi)

    @test vsi.x == xi[1]
    @test vsi.y == xi[2]
    @test vsi.θ == xi[3]
    @test vsi.v == xi[4]

    # Bicycle
    model = BicycleGame(p=3)
    xi = @SVector [1.0, 2.0, sqrt(3)/2, 0.5]
    vsi = standardize(model, xi)

    @test vsi.x == xi[1]
    @test vsi.y == xi[2]
    @test vsi.θ == xi[4]
    @test vsi.v == xi[3]


    # Test specialize
    T = Float64
    p = 3
    model = DoubleIntegratorGame(p=p)
    x = rand(SVector{model.n,T})
    vs = standardize(model, x)
    x_ = specialize(model, vs)
    @test norm(x - x_, 1) < 1e-10

    p = 3
    model = UnicycleGame(p=p)
    x = rand(SVector{model.n,T})
    vs = standardize(model, x)
    x_ = specialize(model, vs)
    @test norm(x - x_, 1) < 1e-10

    p = 3
    model = BicycleGame(p=p)
    x = rand(SVector{model.n,T})
    vs = standardize(model, x)
    x_ = specialize(model, vs)
    @test norm(x - x_, 1) < 1e-10


end
