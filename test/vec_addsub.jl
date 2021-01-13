@testset "vec sub/add" begin
    n = rand(10:100)

    @testset "numeric types" begin
        numtypes = [Float64, Float32, Int64, Int32]
        for T in numtypes
            vectypes = [Vector, MVector{n,T}, SizedVector{n,T}]
            for  V in vectypes
                if V == Vector
                    a = rand(T,n)
                    b = rand(T,n)
                else
                    a = rand(V)
                    b = rand(V)
                end
                b0 = copy(b)
                c = a + b
                @test vec_add!(a,b) === a  # make sure it returns `a`
                @test all(a .≈ c)          # check output
                @test b == b0              # make sure `b` isn't modified
                @test eltype(a) == T

                c = a - b
                @test vec_sub!(a,b) === a  # make sure it returns `a`
                @test all(a .≈ c)          # check output
                @test b == b0              # make sure `b` isn't modified
                @test eltype(a) == T
            end
        end
    end

    @testset "vectors" begin
        a = [rand(rand(n:2n)) for _ = 1:n]
        b = [rand(l) for l in length.(a)]

        # Try with vectors
        c = a + b
        vec_add!(a,b)
        @test c ≈ a
        @test length.(c) == length.(a)

        c = a - b
        vec_sub!(a,b)
        @test c ≈ a
        @test length.(c) == length.(a)
    end

    @testset "exceptions" begin
        # Check errors
        a = rand(n)
        b = rand(2n)
        @test_throws AssertionError vec_add!(a,b)
        @test_throws AssertionError vec_sub!(a,b)
    end

    # Test greet
    f = open("out", create=true, read=true, write=true)
    redirect_stdout(JuliaTemplateRepo.greet, f)
    seekstart(f)
    @test readline(f) == "Hello World!"
    close(f)
    rm("out")
end
