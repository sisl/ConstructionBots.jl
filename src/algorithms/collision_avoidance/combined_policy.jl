@with_kw mutable struct CombinedPolicy <: DeconflictStrategy
    name::String="RVO-TangentBug-PotentialFields"
    # Note: Policies can be combined in a specified order:
    policies::Vector{DeconflictStrategy}=[
        ReciprocalVelocityObstacle(),
        TangentBugPolicy(),
        PotentialFields(),
    ]
    dt::Float64 = 1/40.0
    neighbor_distance::Float64 = DEFAULT_NEIGHBOR_DISTANCE
    max_neighbors::Int = 5
    horizon::Float64 = 2.0
    horizon_obst::Float64 = 1.0
    default_radius::Float64 = 0.5
    max_speed::Float64 = DEFAULT_MAX_SPEED
    default_velocity::Tuple{Float64, Float64} = (0.0, 0.0)
end

function perform_twist_deconfliction(c::CombinedPolicy, params)
    sim = rvo_global_sim()
    # params should contain information about all agents to be updated
    for (id, agent_params) in params
        pos = (agent_params.x, agent_params.y)
        vel = (agent_params.vx, agent_params.vy)
        idx = rvo_get_agent_idx(id)
        sim.setAgentPosition(idx, pos)
        sim.setAgentPrefVelocity(idx, vel)
    end
    sim.doStep()  # Update simulation to next time step
    for (id, _) in params
        idx = rvo_get_agent_idx(id)
        new_vel = sim.getAgentVelocity(idx)
        # Apply new_vel to the agent in simulation environment
    end
end

function update_env_with_deconfliction(c::CombinedPolicy, scene_tree, env)
    rvo_set_new_sim!(env)
    rvo_add_agents!(scene_tree)
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

function update_simulation!(c::CombinedPolicy, env)
    update_rvo_sim!(env)
end

function update_agent_position_in_sim!(c::CombinedPolicy, env, agent)
    pt = get_agent_position(env.deconfliction_type, agent)
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

function set_agent_properties(c::CombinedPolicy)
    set_rvo_default_neighbor_distance!(16 * default_robot_radius())
    set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())
end

function set_agent_priority!(c::CombinedPolicy, env, agent)
    return set_rvo_priority!(env, agent)
end

function get_agent_position(c::CombinedPolicy, agent)
    rvo_idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentPosition(rvo_idx)
end

function set_agent_position!(c::CombinedPolicy, agent, pos)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPosition(idx, (pos[1], pos[2]))
end

function set_agent_max_speed!(c::CombinedPolicy, agent, speed=get_agent_max_speed(agent))
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentMaxSpeed(idx, speed)
end

function get_agent_velocity(c::CombinedPolicy, agent)
    idx = rvo_get_agent_idx(agent)
    rvo_global_sim().getAgentVelocity(idx)
end

function get_agent_pref_velocity(c::CombinedPolicy, agent)
    idx = rvo_get_agent_idx(entity(agent))
    return rvo_global_sim().getAgentPrefVelocity(idx)
end

function set_agent_pref_velocity!(c::CombinedPolicy, agent, desired_velocity)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPrefVelocity(idx, (desired_velocity[1], desired_velocity[2]))
end

function get_agent_alpha(c::CombinedPolicy, agent)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentAlpha(idx)
end

function set_agent_alpha!(c::CombinedPolicy, agent, alpha=0.5)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentAlpha(idx, alpha)
end
