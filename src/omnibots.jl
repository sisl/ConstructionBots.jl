# """
#     TimeStampedState{S,T}
#
# Wrapper for states that contain a time stamp
# """
# struct TimeStampedState{S,T}
#     s::S
#     t::T
# end
# struct CountdownState{S}
#     s::S
#     counter::Int
# end

abstract type AbstractRigidBodyState{N} end
"""
    PlanarRigidBodyState{N}

Exists in 3D, but can only rotate about the Z axis
"""
@with_kw_noshow struct PlanarRigidBodyState{N} <: AbstractRigidBodyState{N}
    pos::SVector{N,Float64} = zero(SVector{N,Float64})
    vel::SVector{N,Float64} = zero(SVector{N,Float64})
    θ::Float64              = 0.0
    ω::Float64              = 0.0
end
pos(s::AbstractRigidBodyState) = s.pos
vel(s::AbstractRigidBodyState) = s.vel
orientation(a::PlanarRigidBodyState) = s.θ
yaw_rate(a::PlanarRigidBodyState) = s.ω
Base.string(s::PlanarRigidBodyState) = string(typeof(s),"(\n  pos=",[s.pos...],",\n  vel=",[s.vel...],",\n  θ=",s.θ,"\n  ω=",s.ω)
Base.show(io::IO,s::PlanarRigidBodyState) = print(io,string(s))

"""
    PlanarRigidBodyAction{N}

Represents the action of an omnidirectional robot. Assumes that the robot can
Fields:
- positional acceleration `accel`
- angular acceleration `α`
"""
@with_kw_noshow struct PlanarRigidBodyAction{N}
    accel::SVector{N,Float64}   = zero(SVector{N,Float64})
    α::Float64                  = 0.0
    dt::Float64                 = 1.0
end
get_dt(a::PlanarRigidBodyAction)  = a.dt
Base.string(a::PlanarRigidBodyAction) = string(typeof(a),"(\n  accel=",[a.accel...],",\n  α=",a.α,")")
Base.show(io::IO,a::PlanarRigidBodyAction) = print(io,string(a))

integrate_linear(dt,p,v=0.0,a=0.0) = p .+ v*dt .+ 0.5*a*dt^2
integrate_polynomial(dt,coeffs) = [a*(1.0/factorial(i-1))*dt^(i-1) for (i,a) in enumerate(coeffs)]
function get_next_state(s::PlanarRigidBodyState,a::PlanarRigidBodyAction)
    PlanarRigidBodyState(
        pos = integrate_linear(a.dt,s.pos,s.vel,a.accel),
        vel = integrate_linear(a.dt,s.vel,a.accel),
        θ   = wrap_to_pi(integrate_linear(a.dt,s.θ,s.ω,a.α)),
        ω   = integrate_linear(a.dt,s.ω,a.α),
    )
end

"""
    CompositeState{T}

Type for storing multiple distinct states.
"""
struct CompositeState{T}
    states::T
end
"""
    CompositeAction{T}

Type for storing multiple distinct states.
"""
struct CompositeAction{T}
    actions::T
end

function get_next_state(s::CompositeState{T},a::CompositeAction) where {T}
    CompositeState(
        T([get_next_state(state,action) for (state,action) in zip(s.states,a.actions)])
    )
end

export
    OmniBotState
    # OmniBotState,
    # OmniBotAction

"""
    AbstractOmniBotState{N}

Abstract type representing the state of an omnidirectional robot.
"""
abstract type AbstractOmniBotState{N} end
get_t(s::AbstractOmniBotState) = s.t
Base.show(io::IO,s::AbstractOmniBotState) = print(io,
    typeof(s),"(\n  pos=",[s.pos...],",\n  vel=",[s.vel...],",\n  θ=",s.θ,"\n  ω=",s.ω)

"""
    OmniBotState{N}

Represents the state of an omnidirectional robot. Assumes that the robot can
only rotate along one axis.
Fields:
- position `pos`
- velocity `vel`
- angular orientation `θ`
- angular velocity `ω`
"""
@with_kw_noshow struct OmniBotState{N} <: AbstractOmniBotState{N}
    s::PlanarRigidBodyState{N}  = PlanarRigidBodyState{N}()
    t::Float64                  = 0.0
end
function get_next_state(s::S,a::PlanarRigidBodyAction) where {S<:OmniBotState}
    S(get_next_state(s.s,a), s.t + a.dt)
end

# """
#     AbstractOmniBotAction{N}
#
# Abstract type representing the action of an omnidirectional robot.
# """
# abstract type AbstractOmniBotAction{N} end
# get_dt(a::AbstractOmniBotAction)  = a.dt
# Base.show(io::IO,a::AbstractOmniBotAction) = print(io,
#     typeof(a),"(\n  accel=",[a.accel...],",\n  α=",a.α,")")
#
# """
#     OmniBotAction{N}
#
# Represents the action of an omnidirectional robot. Assumes that the robot can
# Fields:
# - positional acceleration `accel`
# - angular acceleration `α`
# """
# @with_kw_noshow struct OmniBotAction{N} <: AbstractOmniBotAction{N}
#     accel::SVector{N,Float64}   = zero(SVector{N,Float64})
#     α::Float64                  = 0.0
#     dt::Float64                 = 1.0
# end
# # function get_next_state(s::OmniBotState,a::AbstractOmniBotAction)
# function get_next_state(s::S,a::PlanarRigidBodyAction) where {S<:OmniBotState}
#     S(get_next_state(s.s,a),s.t + a.dt)
# end

abstract type AbstractOmniBotModel end

"""
    DiscretizedOmnibotModel

A type for defining a discretized OmniBot action space.
The default model results in the robot being able to move at a single velocity
in any of the cardinal directions, and to rotate (in place only) so as to face
in any of the cardinal directions.
"""
@with_kw_noshow struct DiscretizedOmnibotModel
    ϵ = 1e-4
    v_max = 1.0
    candidate_accels = [one_hot(3,1),one_hot(3,2),-one_hot(3,1),-one_hot(3,2)]
    allow_perpendicular_accel=false
    allow_translating_rotation=false
    ω_max = π/2
    candidate_α_list = [π/2,-π/2]
    dt = 1.0
end

"""
    get_possible_actions(model,s::OmniBotState{N}) where {N}

The robot may only rotate if it has no cartesian velocity. It can only
accelerate +/- the maximum accel, and must remain within velocity bounds (this
applies to rotational and translational acceleration). It can only accelerate
laterally if not already moving longitudinally, and vice versa.
"""
function get_possible_actions(model,s::OmniBotState{N}) where {N}
    accels = Vector{SVector{N,Float64}}([[0.0,0.0,0.0]])
    α_list = [0.0]
    # Prevent acceleration perpendicular direction of movement
    if abs(s.ω) < model.ϵ # No accel if turning
        for accel in model.candidate_accels
            if norm(s.vel × accel) < model.ϵ || ~model.allow_perpendicular_accel
                if norm(s.vel + accel*model.dt) < model.v_max + model.ϵ
                    push!(accels, accel)
                end
            end
        end
    end
    # Prevent angular acceleration if moving
    if norm(s.vel) < model.ϵ || model.allow_translating_rotation
        for α in model.candidate_α_list
            if abs(s.ω + α*model.dt) < model.ω_max + model.ϵ
                push!(α_list,α)
            end
        end
    end
    actions = Vector{PlanarRigidBodyAction{N}}()
    for accel in accels
        if norm(accel) < model.ϵ
            for α in α_list
                push!(actions,PlanarRigidBodyAction{N}(accel=accel,α=α,dt=model.dt))
            end
        else
            α = 0.0
            push!(actions,PlanarRigidBodyAction{N}(accel=accel,α=α,dt=model.dt))
        end
    end
    return actions
end

# Factory Bot

"""
    PlatformState

Describes the state of a vertical platform.
"""
@with_kw_noshow struct PlatformState
    pos::Float64 = 0.0
    vel::Float64 = 0.0
    t::Float64   = 1.0
end
@with_kw_noshow struct PlatformAction
    accel::Float64 = 0.0 # acceleration of platform
    dt::Float64 = 1.0
end
function get_next_state(s::PlatformState,a::PlatformAction)
    PlatformState(
        pos = integrate_linear(a.dt,s.pos,s.vel,a.accel),
        vel = integrate_linear(a.dt,s.vel,a.accel),
    )
end
@with_kw_noshow struct PlatformOmniBotState{N} <: AbstractOmniBotState{N}
    state::OmniBotState{N}          = OmniBotState{N}()
    platform_state::PlatformState   = PlatformState()
    t::Float64                      = 0.0
end
@with_kw_noshow struct PlatformPlanarRigidBodyAction{N} <: AbstractOmniBotAction{N}
    action::PlanarRigidBodyAction{N} = PlanarRigidBodyAction{N}()
    platform_action::PlatformAction = PlatformAction()
end
function get_next_state(s::PlatformOmniBotState,a::PlatformOmniBotAction)
    PlatformOmniBotState(
        state = get_next_state(s.state,a.action),
        platform_state = get_next_state(s.platform_state,s.platform_action)
    )
end
