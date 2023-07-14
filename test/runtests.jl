using ConstructionBots
using Test
using Logging
using Random

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))

# Fix randomness during tests
Random.seed!(0)

@inline function array_isapprox(x::AbstractArray{F},
                  y::AbstractArray{F};
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    # Easy check on matching size
    if length(x) != length(y)
        return false
    end

    for (a,b) in zip(x,y)
        if !isapprox(a,b, rtol=rtol, atol=atol)
            return false
        end
    end
    return true
end

# Check if array equals a single value
@inline function array_isapprox(x::AbstractArray{F},
                  y::F;
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    for a in x
        if !isapprox(a, y, rtol=rtol, atol=atol)
            return false
        end
    end
    return true
end

# Define package tests
@time @testset "ConstructionBots Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    # @time @testset "ConstructionBots.Omnibots" begin
    #     include(joinpath(testdir, "test_omnibots.jl"))
    # end
end
