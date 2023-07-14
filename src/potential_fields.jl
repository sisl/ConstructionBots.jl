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

mutable struct PairwisePotential
    f
    scale
end
(p::PairwisePotential)(x1,x2,r1,r2) = p.f(x1,x2,r1,r2)

"""
    repulsion_potential(x,r,x2,r2;

Combines a cone potential and a 1/||x|| barrier potential for pairwise
repulsion.
"""
function repulsion_potential(x,r,x2,r2;
        dr = 2*HierarchicalGeometry.default_robot_radius(),
        # fillet_radius=0.5*default_robot_radius(),
    )
    dx = x .- x2
    R = r + r2
    # # # cone
    f1 = max(0.0,R+dr - norm(dx))
    # barrier
    f2 = max(0,1/(norm(dx)-R) - 1/(dr))
    return f1+f2
end

@with_kw mutable struct PotentialFieldController
    env                             = nothing
    agent                           = nothing
    node                            = nothing
    agent_radius                    = 1.0
    vmax                            = 1.0
    # for modifying potentials
    dist_to_nearest_active_agent    = Inf               # track distance to closest agent
    max_buffer_radius               = 2.0*agent_radius  # maximum buffer to add for potential
    buffer_radius                   = max_buffer_radius # how much buffer to add for potential
    interaction_radius              = 20*ROBOT_RADIUS
    # potentials between all pairs of robots
    pairwise_potentials             = (x,r,xp,rp)->repulsion_potential(x,r,xp,rp)
    # environment potentials (may depend on other agent's states)
    static_potentials               = (x,r)->0.0
end
function dist_to_nearest_active_agent(policy::PotentialFieldController)
    env = policy.env
    @unpack scene_tree, sched, cache = env
    agent = policy.agent
    pos = HierarchicalGeometry.project_to_2d(global_transform(agent).translation)
    shortest_dist = Inf
    id = nothing
    for (other_id,other_p) in env.agent_policies
        other_policy=other_p.dispersion_policy
        if !(other_policy === policy)
            other_node = other_policy.node
            if (parent_build_step_is_active(other_node,env) && cargo_ready_for_pickup(other_node,env))
                other_agent = other_policy.agent
                other_rad = other_policy.agent_radius
                other_pos = HierarchicalGeometry.project_to_2d(global_transform(other_agent).translation)
                # other_rad = HierarchicalGeometry.get_radius(get_base_geom(other_agent,HypersphereKey()))
                dist = norm(pos - other_pos) - (other_rad + policy.agent_radius)
                if dist < shortest_dist
                    shortest_dist = min(dist,shortest_dist)
                    id = node_id(other_agent)
                end
            end
        end
    end
    return id, shortest_dist
end
function update_dist_to_nearest_active_agent!(policy::PotentialFieldController)
    id, dist = dist_to_nearest_active_agent(policy)
    policy.dist_to_nearest_active_agent = dist
end
function update_buffer_radius!(
        policy::PotentialFieldController,
        r=min(policy.max_buffer_radius, HierarchicalGeometry.default_robot_radius()/policy.dist_to_nearest_active_agent),
    )
    policy.buffer_radius = r
end
# pairwise_potential_field(policy)
function static_potential_gradient(c::PotentialFieldController,pos)
    ForwardDiff.gradient(x->c.static_potentials(x,c.agent_radius),pos)
end
function pairwise_potential_gradient(c::PotentialFieldController,pos,other_pos,other_radius;dr=1.0)
    ForwardDiff.gradient(
        x->c.pairwise_potentials(
            x,
            c.agent_radius,
            other_pos,
            other_radius,
            ;
            dr=dr
            ),
        pos)
end

function clip_velocity(vel,vmax)
    if norm(vel) > vmax
        v = vmax*normalize(vel)
    else
        v = vel
    end
    return v
end

function pairwise_potential_scale(policy::PotentialFieldController,α1,α2)
    if α1 < α2
        return 0.0 # precedence--no push
    elseif α1 == 0
        return α2 / (α1 + min(α2,0.05))
    else
        return α2 / α1
    end
end

"""
    pairwise_potential_width(policy::PotentialFieldController,α1,α2)

How much extra width to add to pairwise potential.
"""
function pairwise_potential_width(policy::PotentialFieldController,α1,α2)
    if α1 < α2
        return 0.0 # no push
    elseif α1 == 0
        return HierarchicalGeometry.default_robot_radius()
    else
        return 0.5*HierarchicalGeometry.default_robot_radius()
    end
end

function compute_potential_gradient!(policy::PotentialFieldController,pos)
    @unpack scene_tree, sched = policy.env
    agent = policy.agent
    dp = static_potential_gradient(policy,pos)
    α1 = rvo_get_agent_alpha(agent)
    for other_agent in rvo_active_agents(scene_tree)
        if !(agent === other_agent)
            α2 = rvo_get_agent_alpha(other_agent)
            if α1 < α2 # can't be pushed by other agent
                continue
            end
            other_policy = policy.env.agent_policies[node_id(other_agent)].dispersion_policy
            other_rad = other_policy.agent_radius
            # other_rad = HierarchicalGeometry.get_radius(get_base_geom(other_agent,HypersphereKey()))
            # scale = pairwise_potential_scale(policy,α1,α2)
            scale = 1.0
            other_pos = HierarchicalGeometry.project_to_2d(global_transform(other_agent).translation)
            if norm(pos - other_pos) <= policy.interaction_radius
                dp = dp .+ scale*pairwise_potential_gradient(
                    policy,
                    pos,
                    other_pos,
                    other_rad; # + dilate based on priority?
                    dr = other_policy.buffer_radius,
                    )
            end
        end
    end
    return dp
end
function compute_velocity_command!(policy::PotentialFieldController,pos)
    dp = compute_potential_gradient!(policy::PotentialFieldController,pos)
    vel = clip_velocity(-1.0 * dp, policy.vmax)
    return vel
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
