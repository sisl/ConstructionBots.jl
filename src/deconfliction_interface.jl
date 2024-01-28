# TODO(tashakim): Import rvo_interface and other deconfliction 
# algorithm implementations

# TODO(tashakim): Define DeconflictionType enum to pass in

abstract type DeconflictStrategy end

# Update the simulation environment by specifying new agent properties.
function update_simulation_environment(deconfliction_type)
    if deconfliction_type == "rvo"
        return rvo_set_new_sim!()
    else
        println("No simulation update required: $deconfliction_type")
    end
end

# Add agents to simulation based on the deconfliction algorithm used.
function add_agents_to_simulation(scene_tree, deconfliction_type)
    if deconfliction_type == "rvo"
        return rvo_add_agents!(scene_tree)
    else
        println("No new agents to add: $deconfliction_type")
    end
end

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
function set_agent_priority(env::PlannerEnv, node)
    if deconfliction_type == "rvo"
        return set_rvo_priority!(env, node)
    else
        println("No agent priority to update: $deconfliction_type")
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, deconfliction_type)
    if deconfliction_type == "rvo"
        return get_rvo_max_speed(node)  # RVOInterface.get_rvo_max_speed(node)
    else
        # Handle other deconfliction types or throw an error
        error("Unsupported deconfliction type: $deconfliction_type")
    end
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
