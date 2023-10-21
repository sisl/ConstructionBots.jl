using HierarchicalGeometry
using GeometryBasics
using CoordinateTransformations
using Rotations
using Polyhedra
using Graphs
using GraphUtils
using LazySets
using StaticArrays
using LinearAlgebra

using Test, Logging
# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))

# Check equality of two arrays
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

@testset "HierarchicalGeometry.jl" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    @time @testset "HierarchicalGeometry.Overapproximation" begin
        include(joinpath(testdir, "test_approximations.jl"))
    end
    @time @testset "HierarchicalGeometry.Transformations" begin
        include(joinpath(testdir, "test_transformations.jl"))
    end
end
