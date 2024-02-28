abstract type DeconflictStrategy end

# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.
include("reciprocal_velocity_obstacle.jl")
include("tangent_bug_policy.jl")
include("potential_fields.jl")
include("custom_policy.jl")

const supported_deconfliction_options = Dict(
    :RVO => ReciprocalVelocityObstacle()
)

# TODO(tashakim): Refine method to set default agent properties 
# that are dependent on the type of deconflict strategies used. 
# I.e., this method should replace rvo_default_neighbor_distance,
# rvo_default_min_neighbor_distance etc.
function set_agent_properties(deconfliction_type)
    if deconfliction_type isa ReciprocalVelocityObstacle
        # TODO(tashakim): Consider if these fields should be part of 
        # DeconflictStrategy type.
        deconfliction_type.neighbor_distance = (16 * default_robot_radius())
        set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())
    else
        @debug "No agent properties set for deconfliction type: 
        $(join(env.deconfliction_type.name, ", "))"
    end
end

function update_env_with_deconfliction(env)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_vmax(n, env)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = env.deconfliction_type isa TangentBugPolicy ?
                                 TangentBugPolicy(
                    dt = env.dt,
                    vmax = vmax,
                    agent_radius = agent_radius,
                ) : nothing,
                dispersion_policy = env.deconfliction_type isa Dispersion ?
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

# Adjust the priority of agents to manage their interactions and avoid
# potential gridlocks. 
# TODO(tashakim): assess whether deconflict_strategies can be passed in for 
# this method.
function set_agent_priority!(env, node)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return set_rvo_priority!(env, node)
    else
        @debug "No agent priority to update for deconfliction type: 
        $(join(env.deconfliction_type.name, ", "))"
    end
end

function get_agent_position(env, agent)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return rvo_get_agent_position(agent)
    end
end

get_agent_max_speed(::RobotNode) = DEFAULT_MAX_SPEED
function get_agent_max_speed(node)
    rect = get_base_geom(node, HyperrectangleKey())
    vol = LazySets.volume(rect)  # speed limited by volume
    vmax = DEFAULT_MAX_SPEED
    delta_v = vol * DEFAULT_MAX_SPEED_VOLUME_FACTOR
    return max(vmax - delta_v, DEFAULT_MIN_MAX_SPEED)
end

function set_agent_max_speed!(env, node, speed)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return rvo_set_agent_max_speed!(node, speed)
    else
        @debug "No agent max speed to update for deconfliction type: 
        $(join(env.deconfliction_type.name, ", "))"
    end
end

function set_agent_pref_velocity!(env, node, desired_velocity)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return rvo_set_agent_pref_velocity!(node, desired_velocity)
    else
        if matches_template(Union{RobotGo, TransportUnitGo}, node)
            node.desired_twist = desired_velocity
        end
    end
end

function get_agent_pref_velocity(env, agent)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return rvo_get_agent_pref_velocity(entity(agent))
    else
        if matches_template(Union{RobotGo, TransportUnitGo}, agent)
            return agent.node.desired_twist
        end
    end
end

function set_agent_alpha!(env, node, alpha = 0.5)
    if env.deconfliction_type isa ReciprocalVelocityObstacle
        return rvo_set_agent_alpha!(node, alpha)
    else
        node.alpha = alpha
    end
end

# Return the maximum speed of a node based on its type and volume.
function get_vmax(node, env)
    # TODO(tashakim): Computing vmax is necessary for simulation to run, but 
    # currently relies on RVO fields. A new method should be implemented that
    # enables computing vmax without using any RVO fields so that vmax can be 
    # computed for any deconfliction strategy.
    @debug "Maximum speed of node for deconfliction strategy 
    $(env.deconfliction_type.name: vmax)"
    vmax = get_agent_max_speed(node)
    return vmax
end

# TODO(tashakim): Add additional methods for minimal functionality, as appropriate
