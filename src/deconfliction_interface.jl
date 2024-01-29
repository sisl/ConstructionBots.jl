# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.

# TODO(tashakim): Replace `deconfliction_type` from an indicator string
# to a DeconflictionStrategy type after redefining deconfliction common
# methods, e.g. update_velocity(deconfliction_type).

abstract type DeconflictStrategy end

function update_velocity(env, deconfliction_type)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_vmax(n, deconfliction_type)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = in(:TangentBugPolicy, deconfliction_type) ? 
                    TangentBugPolicy(
                        dt = env.dt, 
                        vmax = vmax, 
                        agent_radius = agent_radius
                    ) : nothing,
                dispersion_policy = in(:Dispersion, deconfliction_type) ? 
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
function update_simulation_environment(deconfliction_type)
    if in(:RVO, deconfliction_type)
        return rvo_set_new_sim!()
    else
        println("No simulation update required for deconfliction strategy: ",
            join(symbols_array, ", "))
    end
end

# Add agents to simulation based on the deconfliction algorithm used.
function add_agents_to_simulation!(scene_tree, deconfliction_type)
    if in(:RVO, deconfliction_type)
        return rvo_add_agents!(scene_tree)
    else
        println("No new agents to add for deconfliction strategy: ", 
            join(symbols_array, ", "))
    end
end

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
# TODO(tashakim): assess whether deconfliction_type can be passed in for 
# this method.
function set_agent_priority(env::PlannerEnv, node)
    if in(:RVO, deconfliction_type)
        return set_rvo_priority!(env, node)
    else
        println("No agent priority to update for deconfliction strategy: ", 
            join(symbols_array, ", "))
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, deconfliction_type)
    # TODO(tashakim): Computing vmax is necessary for simulation (See line 566 
    # of full_demo.jl), but currently relies on RVO fields. A new method should
    # be implemented that enables computing vmax without using any RVO fields 
    # so that vmax can be computed for any deconfliction type.

    # if in(:RVO, deconfliction_type)
    vmax = get_rvo_max_speed(node)
    # end

    # Debugging: println("Maximum speed of node for deconfliction strategy $deconfliction_type: $vmax")
    return vmax
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
