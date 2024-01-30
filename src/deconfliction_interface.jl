# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.

# TODO(tashakim): Replace `deconflict_strategies` from an indicator string
# to a Vector{DeconflictionStrategy} type after redefining deconfliction common
# methods, e.g. update_velocity(deconflict_strategies).

abstract type DeconflictStrategy end

# TODO(tashakim): refine method to set default agent properties 
# that are dependent on the type of deconflict strategies used. 
# I.e., this method should replace rvo_default_neighbor_distance,
# rvo_default_min_neighbor_distance etc.
function set_agent_properties(deconflict_strategies)
    if in(:RVO, deconflict_strategies)
        # TODO(tashakim): figure out if these fields should be part of AgentType
        # or DeconflictStrategy type, then update accordingly.
        set_rvo_default_neighbor_distance!(16 * default_robot_radius())
        set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())
    else
        println("No agent properties set for deconfliction strategies: ",
        join(deconflict_strategies, ", "))
    end
end

# TODO(tashakim): take in AgentType parameter to update velocity
function update_velocity(env, deconflict_strategies)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_vmax(n, deconflict_strategies)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = in(:TangentBugPolicy, deconflict_strategies) ? 
                    TangentBugPolicy(
                        dt = env.dt, 
                        vmax = vmax, 
                        agent_radius = agent_radius
                    ) : nothing,
                dispersion_policy = in(:Dispersion, deconflict_strategies) ? 
                    ConstructionBots.PotentialFieldController(
                        agent = n,
                        node = node,
                        agent_radius = agent_radius,
                        vmax = vmax,
                        max_buffer_radius = 2.5 * agent_radius,
                        interaction_radius = 15 * agent_radius,
                        static_potentials = (x, r) -> 0.0,
                        pairwise_potentials = ConstructionBots.repulsion_potential
                    ) : nothing
            )
        end
    end
end

# Update the simulation environment by specifying new agent properties.
function update_simulation_environment(deconflict_strategies)
    if in(:RVO, deconflict_strategies)
        rvo_set_new_sim!()
    else
        println("No simulation update required for deconfliction strategy: ",
            join(deconflict_strategies, ", "))
    end
end

# Add agents to simulation based on the deconfliction algorithm used.
function add_agents_to_simulation!(scene_tree, deconflict_strategies)
    if in(:RVO, deconflict_strategies)
        return rvo_add_agents!(scene_tree)
    else
        println("No new agents to add for deconfliction strategy: ", 
            join(deconflict_strategies, ", "))
    end
end

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
# TODO(tashakim): assess whether deconflict_strategies can be passed in for 
# this method.
function set_agent_priority(env::PlannerEnv, node)
    if in(:RVO, deconflict_strategies)
        return set_rvo_priority!(env, node)
    else
        println("No agent priority to update for deconfliction strategy: ", 
            join(deconflict_strategies, ", "))
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, deconflict_strategies)
    # TODO(tashakim): Computing vmax is necessary for simulation (See line 566 
    # of full_demo.jl), but currently relies on RVO fields. A new method should
    # be implemented that enables computing vmax without using any RVO fields 
    # so that vmax can be computed for any deconfliction strategy.

    # if in(:RVO, deconflict_strategies)
    vmax = get_rvo_max_speed(node)
    # end

    # Debugging: println("Maximum speed of node for deconfliction strategy $deconflict_strategies: $vmax")
    return vmax
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
