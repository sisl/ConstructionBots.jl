using LinearAlgebra
using Colors
using GeometryBasics
using ForwardDiff
using Parameters
using GraphUtils


abstract type Potential end
potential(v::Vector,x) = sum(potential(p,x) for p in v)
potential_gradient(v,x) = ForwardDiff.gradient(x->potential(v,x),x)
potential_gradient(v::Vector,x) = sum(potential_gradient(p,x) for p in v)
(p::Potential)(x) = potential(p,x)


"""
    ConePotential <: Potential

Produces a potential function in the shape of a cone.
"""
mutable struct ConePotential <: Potential
    c # coefficient
    R # radius
    x # center
end
potential(p::ConePotential,x) = p.c*min(p.R,norm(p.x-x))

"""
    HelicalPotential
"""
mutable struct HelicalPotential
    c # coefficient
    x # center
    θ # desired angle
end
function potential(p::HelicalPotential,x)
    ψ = atan(x[2],x[1])
    Δθ = wrap_to_pi(angular_difference(θ,ψ))
    return p.c * Δθ
end

"""
    TentPotential <: Potential

Produces a potential function in the shape of a tent
"""
mutable struct TentPotential <: Potential
    c # coefficient
    R # radius
    x # start
    g # goal
end
function potential(p::TentPotential,x)
    dx = x - p.x
    v = p.g - p.x
    xp = dot(dx,v)*v / norm(v)^2
    if 0.0 < dot(xp,normalize(v)) < norm(v) 
        return p.c*min(p.R,norm(dx-xp))
    else
        return p.c*min(p.R,min(norm(x-p.x),norm(x-(p.g))))
    end
end

mutable struct TransformPotential <: Potential
    p # potential
    f # function
end
potential(p::TransformPotential,x) = p.f(potential(p.p,x))

mutable struct GenericPotential <: Potential
    f # function
end
potential(p::GenericPotential,x) = p.f(x)

mutable struct ScaledPotential
    p # potential
    c # scalar
end
potential(p::ScaledPotential,x) = p.c*potential(p.p,x)

"""
    ClippedPotential

Represents a clipped potential function
"""
mutable struct ClippedPotential <: Potential
    p # potential
    c # coefficient
    R # radius
end
potential(p::ClippedPotential,x) = p.c * max(p.R,potential(p.p,x))

"""
    BarrierPotential

Encodes a potential field formed by inverting the squared value of a child 
potential plus some minimum denominator value
"""
mutable struct BarrierPotential <: Potential
    p # potential
    c # coefficient
    z # minimum denominator
end
potential(p::BarrierPotential,x) = p.c / (p.z + potential(p.p,x)^2)

"""
    RotationalPotential

Encodes a rotational field
"""
mutable struct RotationalPotential
    p # potential
    c # coefficient
    zmax # maximum magnitude
end
# function potential(p::RotationalPotential,x)
#     dx = potential_gradient(p.p,x)
#     v = [0.0,0.0,1.0]
#     @show z = cross(v,[dx...,0.0])[1:2]
#     return p.c * dot(z,x)
# end
function potential_gradient(p::RotationalPotential,x)
    dx = potential_gradient(p.p,x)
    v = [0.0,0.0,1.0]
    dz = cross(v,[dx...,0.0])[1:2]
    if norm(dz) > p.zmax
        return p.zmax * normalize(dz)
    else
        return dz
    end
end

@with_kw mutable struct PotentialController
    x           = nothing # state
    v           = nothing # velocity
    circ_idx    = -1      # index of circle that robot may enter
    goal        = nothing # goal position
    p           = nothing # potential function
    dp          = nothing # gradient of potential function
    radius      = nothing # radius
    vmax        = 1.0
end
ROBOT_RADIUS = 0.5

@with_kw mutable struct GlobalPotentialController
    robot_controllers       = Dict()
    goal_potentials         = Dict()
    circle_potentials       = Dict()
    collision_potentials    = Dict()
    path_potentials         = Dict()
    INTERACTION_RANGE       = 5*ROBOT_RADIUS
end

function update_potential_controllers!(controller)
    for (k,robot) in controller.robot_controllers
        update_potential_controller!(robot,k,controller)
    end
    controller
end
function update_potential_controller!(robot,i,global_controller)
    # i is the robot's index
    potentials = []
    C = global_controller
    # pull robot toward its goal
    push!(potentials,   C.goal_potentials[i])
    # repel robot from circles in which it does not belong
    if robot.circ_idx > 0
        push!(potentials,   C.circle_potentials[robot.circ_idx])
    end
    # allow higher priority robots to push robot out of the way
    for (k,r) in C.robot_controllers
        p = C.path_potentials[k]
        if !(robot === r) && norm(r.x - robot.x) <= C.INTERACTION_RANGE
            if r.circ_idx < robot.circ_idx # replace with priority check
                push!(potentials,p)
            end
        end
    end
    # set potentials
    robot.p = potentials
    # compute local gradient of potential fields
    robot.dp = potential_gradient(robot.p,robot.x)
    # set velocity in direction of gradient descent
    vel = -1.0 * robot.dp 
    if norm(vel) > robot.vmax
        robot.v = robot.vmax*normalize(vel)
    else
        robot.v = vel
    end
    return robot
end