using ConstructionBots

using StaticArrays
using CoordinateTransformations
using GeometryBasics
using Rotations
using HierarchicalGeometry

using Graphs
using GraphUtils

using Test
using Logging

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))

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
@testset "ConstructionBots Tests" begin
    @testset "IDs" begin
        include("test_ids.jl")
    end
    @testset "Potential Fields" begin
        include("test_potential_fields.jl")
    end
    @testset "Twist" begin
        include("test_twist.jl")
    end

    @testset "Demo" begin
        include("test_demo.jl")
    end
end
