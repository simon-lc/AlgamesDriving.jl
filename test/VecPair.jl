@testset "VecPair" begin
    n = rand(10:100)
    @testset "Constructors" begin
        a = rand(n)
        b = rand(n)
        v = VecPair(a, b)
        @test v isa VecPair{Vector{Float64}}

        # Test converion
        b = rand(Float32, n)
        @test_throws MethodError VecPair(a, b)
        @test VecPair{Vector{Float64}}(a, b) isa VecPair{Vector{Float64}}
        @test VecPair{Vector{Float32}}(a, b) isa VecPair{Vector{Float32}}
        b = rand(Int, n)
        @test VecPair{Vector{Float32}}(a, b) isa VecPair{Vector{Float32}}

        @test VecPair{SVector{n,Float32}}(a, b) isa VecPair{SVector{n,Float32}}
        @test VecPair{MVector{n,Float32}}(a, b) isa VecPair{MVector{n,Float32}}

        # Test StaticArray constructor
        numtypes = [Float64, Float32, Int64, Int32]
        for T in numtypes
            a = @SVector rand(T, n)
            b = @MVector rand(T, n)
            v = VecPair(a, b)
            @test v isa VecPair{MVector{n,T}}
            c = a + b
            @test all(v.a .≈ a)
            @test all(v.b .≈ b)
            @test vec_add!(v) === v.a
            @test all(v[1] .≈ c)

            c = (a + b) - b
            @test all(v.b .≈ b)
            @test vec_sub!(v) === v.a
            @test all(v[1] .≈ c)
        end
    end

    # Test norm
    @testset "Operations" begin
        a = rand(n)
        b = rand(n)
        v = VecPair(a, b)
        @test norm(v) ≈ sqrt(norm(a)^2 + norm(b)^2)
        @test norm(v, 1) ≈ norm([norm(a, 1); norm(b, 1)], 1)

        # Test indexing
        @test v[1] == a
        @test v[2] == b
    end

    @testset "Exceptions" begin
        # Test exceptions
        v = VecPair(rand(n), rand(n))
        @test_throws ArgumentError v[3]
        @test_throws ArgumentError v[0]
        a = rand(n)
        b = rand(2n)
        @test_throws AssertionError VecPair(a, b)
        @test_throws AssertionError VecPair(b, a)
        a = @SVector rand(n)
        b = @SVector rand(2n)
        @test_throws MethodError VecPair(a, b)
    end
end
