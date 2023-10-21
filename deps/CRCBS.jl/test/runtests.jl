using Test
using Logging
# Package Under Test
using CRCBS
using JuMP
using Graphs, MetaGraphs
using Parameters
using GraphUtils

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

set_global_verbosity!(0)
# Define package tests
@time @testset "CRCBS.Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    @time @testset "CRCBS.ProblemDefinitionTests" begin
        include(joinpath(testdir, "unit_tests/test_problem_definitions.jl"))
    end
    @time @testset "CRCBS.InterfaceTests" begin
        include(joinpath(testdir, "unit_tests/test_interface.jl"))
    end
    @time @testset "CRCBS.CostModelTests" begin
        include(joinpath(testdir, "unit_tests/test_cost_models.jl"))
    end
    @time @testset "CRCBS.HeuristicTests" begin
        include(joinpath(testdir, "unit_tests/test_heuristics.jl"))
    end
    @time @testset "CRCBS.CommonTests" begin
        include(joinpath(testdir, "unit_tests/test_common.jl"))
    end
    @time @testset "CRCBS.UtilsTests" begin
        include(joinpath(testdir, "unit_tests/test_utils.jl"))
    end
    @time @testset "CRCBS.ImplicitGraphsTests" begin
        include(joinpath(testdir, "unit_tests/test_a_star.jl"))
    end
    @time @testset "CRCBS.SolverUtilsTests" begin
        include(joinpath(testdir, "unit_tests/test_solver_utils.jl"))
    end
    @time @testset "CRCBS.CBSTests" begin
        include(joinpath(testdir, "unit_tests/test_cbs.jl"))
    end
    @time @testset "CRCBS.CBSTests" begin
        include(joinpath(testdir, "unit_tests/test_cbs.jl"))
    end
    @time @testset "CRCBS.MultiStageCBS.ests" begin
        include(joinpath(testdir, "unit_tests/test_multi_stage_cbs.jl"))
    end
    @time @testset "CRCBS.MetaAgentCBSTests" begin
        include(joinpath(testdir, "unit_tests/test_meta_agent_cbs.jl"))
    end
    # @time @testset "CRCBS.FlowProblemsTests" begin
    #     include(joinpath(testdir, "unit_tests/test_flow_problems.jl"))
    # end
end
