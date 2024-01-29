# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.

# TODO(tashakim): Replace `deconfliction_type` from an indicator string
# to a DeconflictionStrategy type after redefining deconfliction common
# methods, e.g. update_velocity(deconfliction_type).

abstract type DeconflictStrategy end

# Update the simulation environment by specifying new agent properties.
function update_simulation_environment(deconfliction_type)
    if deconfliction_type == :RVO
        return rvo_set_new_sim!()
    else
        println("No simulation update required: $deconfliction_type")
    end
end

# Add agents to simulation based on the deconfliction algorithm used.
function add_agents_to_simulation!(scene_tree, deconfliction_type)
    if deconfliction_type == :RVO
        return rvo_add_agents!(scene_tree)
    else
        println("No new agents to add: $deconfliction_type")
    end
end

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
# TODO(tashakim): assess whether deconfliction_type can be passed in for 
# this method.
function set_agent_priority(env::PlannerEnv, node)
    if deconfliction_type == :RVO
        return set_rvo_priority!(env, node)
    else
        println("No agent priority to update: $deconfliction_type")
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, deconfliction_type)
    # TODO(tashakim): Computing vmax is necessary for simulation (See line 566 
    # of full_demo.jl), but currently relies on RVO fields. A new method should
    # be implemented that enables computing vmax without using any RVO fields 
    # so that vmax can be computed for any deconfliction type.

    # if deconfliction_type == :RVO
    vmax = get_rvo_max_speed(node)
    # end

    # Debugging: println("Maximum speed of node for deconfliction type $deconfliction_type: $vmax")
    return vmax
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
