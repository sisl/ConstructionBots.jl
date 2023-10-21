using TaskGraphs
using CRCBS
using Graphs
using GraphUtils
using JuMP
using MathOptInterface
using Random
using TOML
using NBInclude
using Test
using Logging
using Plots

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
        if !isapprox(a,y, rtol=rtol, atol=atol)
            return false
        end
    end
    return true
end

# Add test for jupyternotebook
if !(haskey(ENV, "CI") && ENV["CI"] == "true")
    @nbinclude(joinpath(dirname(@__DIR__), "docs", "TaskGraphTutorial.ipynb"))
end

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
@time @testset "TaskGraphs Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test/unit_tests")
    @time @testset "TaskGraphs.PlanningPredicates" begin
        include(joinpath(testdir, "test_predicates.jl"))
    end
    @time @testset "TaskGraphs.PCCBS" begin
        include(joinpath(testdir, "test_pccbs.jl"))
    end
    @time @testset "TaskGraphs.CoreTests" begin
        include(joinpath(testdir, "test_core.jl"))
    end
    @time @testset "TaskGraphs.ExampleTests" begin
        include(joinpath(testdir, "test_milp_solvers.jl"))
    end
    @time @testset "TaskGraphs.PathPlanning" begin
        include(joinpath(testdir, "test_path_planning.jl"))
    end
    @time @testset "TaskGraphs.Solvers" begin
        include(joinpath(testdir, "test_solver.jl"))
    end
    @time @testset "TaskGraphs.PIBT" begin
        include(joinpath(testdir, "test_pibt.jl"))
    end
    @time @testset "TaskGraphs.Replanning" begin
        include(joinpath(testdir, "test_replanning.jl"))
    end
end
