using ConstructionBots

# Utility libraries
using Logging
using Test
# Geometry, graphs, and mathematical operations
using CoordinateTransformations
using GeometryBasics
using Graphs
using LazySets
using LinearAlgebra
using Rotations
using StaticArrays

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Warn))

# Check approximate equality between arrays, or array elements
@inline function array_isapprox(
    x::Union{AbstractArray{F},F},
    y::Union{AbstractArray{F},F};
    rtol::F = sqrt(eps(F)),
    atol::F = zero(F),
) where {F<:AbstractFloat}
    # Check if x, y are single values and convert to array if needed
    x_array = isa(x, AbstractArray) ? x : fill(x, length(y))
    y_array = isa(y, AbstractArray) ? y : fill(y, length(x_array))
    # Check matching size
    if length(x_array) != length(y_array)
        return false
    end
    # Compare elements
    for (a, b) in zip(x_array, y_array)
        if !isapprox(a, b, rtol = rtol, atol = atol)
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
    @testset "RVO interface" begin
        include("test_rvo_interface.jl")
    end
    @testset "Demo" begin
        include("test_demo.jl")
    end
end
