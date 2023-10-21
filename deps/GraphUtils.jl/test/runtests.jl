# Packages required for testing
using Test
using Random
using Logging
using Graphs
using TOML
# Package Under Test
using GraphUtils

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))

# Fix randomness during tests
Random.seed!(0)

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
        @test isapprox(a,b, rtol=rtol, atol=atol)
    end
end

# Check if array equals a single value
@inline function array_isapprox(x::AbstractArray{F},
                  y::F;
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    for a in x
        @test isapprox(a, y, rtol=rtol, atol=atol)
    end
end

# Define package tests
@time @testset "GraphUtils Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    @time @testset "GraphUtils.GraphSorting" begin
        include(joinpath(testdir, "test_sorting.jl"))
    end
    @time @testset "GraphUtils.Angles" begin
        include(joinpath(testdir, "test_angles.jl"))
    end
    @time @testset "GraphUtils.Construction" begin
        include(joinpath(testdir, "test_construction.jl"))
    end
    @time @testset "GraphUtils.Connectivity" begin
        include(joinpath(testdir, "test_connectivity.jl"))
    end
    @time @testset "GraphUtils.CubicSplines" begin
        include(joinpath(testdir, "test_cubic_splines.jl"))
    end
    @time @testset "GraphUtils.Graphs" begin
        include(joinpath(testdir, "test_graphs.jl"))
    end
    @time @testset "GraphUtils.Trees" begin
        include(joinpath(testdir, "test_abstract_trees.jl"))
    end
    @time @testset "GraphUtils.CachedElements" begin
        include(joinpath(testdir, "test_cached_element.jl"))
    end
    @time @testset "GraphUtils.AbstractIDs" begin
        include(joinpath(testdir, "test_abstract_ids.jl"))
    end
    @time @testset "GraphUtils.FactoryWorlds" begin
        include(joinpath(testdir, "test_factory_worlds.jl"))
    end
    @time @testset "GraphUtils.FileSystem" begin
        include(joinpath(testdir, "test_filesystem.jl"))
    end
    @time @testset "GraphUtils.Printing" begin
        include(joinpath(testdir, "test_printing.jl"))
    end
    @time @testset "GraphUtils.Logging" begin
        include(joinpath(testdir, "test_logging.jl"))
    end
end
