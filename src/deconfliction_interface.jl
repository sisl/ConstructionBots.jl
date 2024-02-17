# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.

# TODO(tashakim): Replace `deconflict_strategies` from an indicator string
# to a Vector{DeconflictionStrategy} type after redefining deconfliction common
# methods, e.g. update_env_with_deconfliction

abstract type DeconflictStrategy end

# TODO(tashakim): refine method to set default agent properties 
# that are dependent on the type of deconflict strategies used. 
# I.e., this method should replace rvo_default_neighbor_distance,
# rvo_default_min_neighbor_distance etc.
function set_agent_properties(deconflict_strategies)
    if in(:RVO, deconflict_strategies)
        # TODO(tashakim): consider if these fields should be part of 
        # DeconflictStrategy type.
        set_rvo_default_neighbor_distance!(16 * default_robot_radius())
        set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())
    else
        println(
            "No agent properties set for deconfliction strategies: ",
            join(deconflict_strategies, ", "),
        )
    end
end

function update_env_with_deconfliction(env)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_vmax(n, env)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = in(:TangentBugPolicy, env.deconflict_strategies) ?
                                 TangentBugPolicy(
                    dt = env.dt,
                    vmax = vmax,
                    agent_radius = agent_radius,
                ) : nothing,
                dispersion_policy = in(:Dispersion, env.deconflict_strategies) ?
                                    ConstructionBots.PotentialFieldController(
                    agent = n,
                    node = node,
                    agent_radius = agent_radius,
                    vmax = vmax,
                    max_buffer_radius = 2.5 * agent_radius,
                    interaction_radius = 15 * agent_radius,
                    static_potentials = (x, r) -> 0.0,
                    pairwise_potentials = ConstructionBots.repulsion_potential,
                ) : nothing,
            )
        end
    end
end

# Update the simulation environment by specifying new agent properties.
function update_simulation_environment(env)
    if in(:RVO, env.deconflict_strategies)
        rvo_set_new_sim!()
    else
        println(
            "No simulation environment update required for deconfliction
            strategy: ", join(env.deconflict_strategies, ", "),
        )
    end
end

function update_simulation!(env)
    if in(:RVO, env.deconflict_strategies)
        update_rvo_sim!(env)
    else
        println(
            "No simulation update required for deconfliction strategy: ",
            join(env.deconflict_strategies, ", "),
        )
    end
end

function update_agent_position_in_sim!(env, agent)
    if in(:RVO, env.deconflict_strategies)
        pt = rvo_get_agent_position(agent)
        @assert has_parent(agent, agent) "agent $(node_id(agent)) should be its own parent"
        set_local_transform!(agent, CoordinateTransformations.Translation(pt[1], pt[2], 0.0))
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
    else
        println(
            "No agent position updated in simulation for deconfliction 
            strategy: ", join(env.deconflict_strategies, ", "),
        )
    end
end

# Add agents to simulation based on the deconfliction algorithm used.
function add_agents_to_simulation!(scene_tree, env)
    if in(:RVO, env.deconflict_strategies)
        return rvo_add_agents!(scene_tree)
    else
        println(
            "No new agents to add for deconfliction strategy: ",
            join(env.deconflict_strategies, ", "),
        )
    end
end

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
# TODO(tashakim): assess whether deconflict_strategies can be passed in for 
# this method.
function set_agent_priority!(env, node)
    if in(:RVO, env.deconflict_strategies)
        return set_rvo_priority!(env, node)
    else
        println(
            "No agent priority to update for deconfliction strategy: ",
            join(env.deconflict_strategies, ", "),
        )
    end
end

function get_agent_position(env, agent)
    if in(:RVO, env.deconflict_strategies)
        return rvo_get_agent_position(agent)
    end
end

get_agent_max_speed(::RobotNode) = DEFAULT_MAX_SPEED
function get_agent_max_speed(node)
    rect = get_base_geom(node, HyperrectangleKey())
    vol = LazySets.volume(rect)
    # Speed limited by volume
    vmax = DEFAULT_MAX_SPEED
    delta_v = vol * DEFAULT_MAX_SPEED_VOLUME_FACTOR
    return max(vmax - delta_v, DEFAULT_MIN_MAX_SPEED)
end

function set_agent_max_speed!(env, node, speed)
    if in(:RVO, env.deconflict_strategies)
        return rvo_set_agent_max_speed!(node, speed)
    else
        println(
            "No agent max speed to update for deconfliction strategy: ",
            join(env.deconflict_strategies, ", "),
        )
    end
end

function set_agent_pref_velocity!(env, node, desired_velocity)
    if in(:RVO, env.deconflict_strategies)
        return rvo_set_agent_pref_velocity!(node, desired_velocity)
    else
        node.desired_twist = desired_velocity
    end
end

function get_agent_pref_velocity(env, agent)
    if in(:RVO, env.deconflict_strategies)
        return rvo_get_agent_pref_velocity(entity(agent))
    else
        println("node is: ", agent.node)
        return agent.node.desired_twist
    end
end

function set_agent_alpha!(env, node, alpha=0.5)
    if in(:RVO, env.deconflict_strategies)
        return rvo_set_agent_alpha!(node, alpha)
    else
        node.alpha = alpha
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, env)
    # TODO(tashakim): Computing vmax is necessary for simulation (See line 566 
    # of full_demo.jl), but currently relies on RVO fields. A new method should
    # be implemented that enables computing vmax without using any RVO fields 
    # so that vmax can be computed for any deconfliction strategy.

    # if in(:RVO, deconflict_strategies)
    vmax = get_agent_max_speed(node)
    # end

    # Debugging: println("Maximum speed of node for deconfliction strategy $deconflict_strategies: $vmax")
    return vmax
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
