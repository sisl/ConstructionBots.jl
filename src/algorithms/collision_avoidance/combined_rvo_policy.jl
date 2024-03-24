@with_kw mutable struct CombinedRVOPolicy <: DeconflictStrategy
    name::String="RVO-TangentBug-PotentialFields"
    dt::Float64 = 1/40.0
    neighbor_distance::Float64 = DEFAULT_NEIGHBOR_DISTANCE
    min_neighbor_distance::Float64 = DEFAULT_MINIMUM_NEIGHBOR_DISTANCE
    neighborhood_velocity_scale_factor = DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR
    max_neighbors::Int = 5
    horizon::Float64 = 2.0
    horizon_obst::Float64 = 1.0
    default_radius::Float64 = 0.5
    max_speed::Float64 = DEFAULT_MAX_SPEED
    default_velocity::Tuple{Float64, Float64} = (0.0, 0.0)
end

"""
perform_twist_deconfliction(c::CombinedRVOPolicy, env, node) -> Twist

Computes a deconflicted twist for an agent using the Reciprocal Velocity
Obstacle, Tangent Bug Policy, and Potential Fields algorithms (in that order).

# Arguments
- `c::CombinedRVOPolicy`: A deconfliction strategy instance that contains 
necessary parameters.
- `env`: Simulation environment with current state of agents and obstacles.
- `node`: The agent for which the twist is being calculated.

# Returns
- `Twist`: The computed twist that combines linear and angular velocities, 
optimized to achieve goal-oriented movement while avoiding collisions with both
static and dynamic obstacles.
"""
function perform_twist_deconfliction(c::CombinedRVOPolicy, env, node)
    # Apply tangent bug policy
    twist = perform_twist_deconfliction(TangentBugPolicy(), env, node)
    @unpack sched, agent_policies, dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    mode = :not_set
    # Apply traffic thinning
    # Set nominal velocity to zero if close to goal (Note: this is a hack)
    parent_step = get_parent_build_step(sched, node)
    if !(parent_step === nothing)
        countdown = active_build_step_countdown(parent_step.node, env)
        dist_to_goal = norm(goal.translation .- global_transform(agent).translation)
        unit_radius = get_base_geom(entity(node), HypersphereKey()).radius
        if (mode != :EXIT_CIRCLE) && (dist_to_goal < 15 * unit_radius)
            # Ensure the agent doesn't crowd its destination
            if matches_template(TransportUnitGo, node) && countdown >= 1
                twist = Twist(0.0 * twist.vel, twist.ω)
            elseif matches_template(RobotGo, node) && countdown >= 3
                twist = Twist(0.0 * twist.vel, twist.ω)
            end
        end
    end
    # Apply potential fields
    policy = agent_policies[node_id(agent)].dispersion_policy
    # For RobotGo node, ensure that parent assembly is "pickup-able"
    ready_for_pickup = cargo_ready_for_pickup(node, env)
    # TODO: Check if we can use the cached version here
    build_step_active = parent_build_step_is_active(node, env)
    update_dist_to_nearest_active_agent!(policy, env)
    update_buffer_radius!(policy, node, build_step_active, ready_for_pickup)
    if !(build_step_active && ready_for_pickup)
        policy.node = node  # update policy
        nominal_twist = twist  # compute target position
        pos = project_to_2d(global_transform(agent).translation)
        va = nominal_twist.vel[1:2]
        target_pos = pos .+ va * dt
        vb = -1.0 * compute_potential_gradient!(
            policy, ReciprocalVelocityObstacle(), env, pos)  # commanded velocity from current position
        vc = -1.0 * compute_potential_gradient!(
            policy, ReciprocalVelocityObstacle(), env, target_pos)  # commanded velocity from current position
        # Blend the three velocities
        a = 1.0
        b = 1.0
        c = 0.0
        v = (a * va + b * vb + c * vc)
        vel = clip_velocity(v, policy.vmax)
        # Compute goal position
        goal_pt = pos + vel * dt
        goal =
            CoordinateTransformations.Translation(goal_pt..., 0.0) ∘
            CoordinateTransformations.LinearMap(goal.linear)
        twist = compute_twist_from_goal(env, agent, goal, dt)  # nominal twist
    else
        !(policy === nothing)
        policy.dist_to_nearest_active_agent = 0.0
    end
    return twist
end

function update_env_with_deconfliction(c::CombinedRVOPolicy, scene_tree, env)
    rvo_set_new_sim!(c)
    rvo_add_agents!(c, scene_tree)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_agent_max_speed(n)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = TangentBugPolicy(
                    dt = env.dt,
                    vmax = vmax,
                    agent_radius = agent_radius,
                ),
                dispersion_policy = ConstructionBots.PotentialFieldController(
                    agent = n,
                    node = node,
                    agent_radius = agent_radius,
                    vmax = vmax,
                    max_buffer_radius = 2.5 * agent_radius,
                    interaction_radius = 15 * agent_radius,
                    static_potentials = (x, r) -> 0.0,
                    pairwise_potentials = ConstructionBots.repulsion_potential,
                ),
            )
        end
    end
end

function update_simulation!(c::CombinedRVOPolicy, env)
    update_rvo_sim!(c, env)
end

function update_agent_position_in_sim!(c::CombinedRVOPolicy, env, agent)
    pt = get_agent_position(c, agent)
    @assert has_parent(agent, agent) "agent $(node_id(agent)) should be its own parent"
    set_local_transform!(
        agent,
        CoordinateTransformations.Translation(pt[1], pt[2], 0.0),
    )
    if !isapprox(
        norm(global_transform(agent).translation[1:2] .- pt),
        0.0;
        rtol = 1e-6,
        atol = 1e-6,
    )
        @warn "Agent $node_id(agent) should be at $pt but is at 
        $(global_transform(agent).translation[1:2])"
    end
    return global_transform(agent)
end

function set_agent_properties(c::CombinedRVOPolicy)
    c.neighbor_distance = 16 * default_robot_radius()
    c.min_neighbor_distance = 10 * default_robot_radius()
end

function set_agent_priority!(c::CombinedRVOPolicy, env, agent)
    return set_rvo_priority!(env, agent)
end

function get_agent_position(c::CombinedRVOPolicy, agent)
    rvo_idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentPosition(rvo_idx)
end

function set_agent_position!(c::CombinedRVOPolicy, agent, pos)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPosition(idx, (pos[1], pos[2]))
end

function set_agent_max_speed!(c::CombinedRVOPolicy, agent, speed=get_agent_max_speed(agent))
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentMaxSpeed(idx, speed)
end

function get_agent_pref_velocity(c::CombinedRVOPolicy, agent)
    idx = rvo_get_agent_idx(entity(agent))
    return rvo_global_sim().getAgentPrefVelocity(idx)
end

function set_agent_pref_velocity!(c::CombinedRVOPolicy, agent, desired_velocity)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPrefVelocity(idx, (desired_velocity[1], desired_velocity[2]))
end

function get_agent_alpha(c::CombinedRVOPolicy, agent)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentAlpha(idx)
end

function set_agent_alpha!(c::CombinedRVOPolicy, agent, alpha=0.5)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentAlpha(idx, alpha)
end
