abstract type DeconflictStrategy end

# TODO(tashakim): Import deconfliction algorithm implementations once
# complete.
include("reciprocal_velocity_obstacle.jl")
include("tangent_bug_policy.jl")
include("potential_fields.jl")
include("custom_policy.jl")

const supported_deconfliction_options = Dict(
    :RVO => ReciprocalVelocityObstacle(),
    :TangentBugPolicy => TangentBugPolicy(),
    :PotentialFields => PotentialFields(),
    :Nothing => Nothing,
)

# Interface methods
"""
"""
function set_agent_properties(d::DeconflictStrategy)
    @debug "set_agent_properties not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function update_env_with_deconfliction(scene_tree, env)
    update_simulation_environment(env)
    add_agents_to_simulation!(scene_tree, env)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_agent_max_speed(n)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = in(:TangentBugPolicy, env.deconflict_strategies) ?
                                 TangentBugPolicy(
                    dt = env.dt,
                    vmax = vmax,
                    agent_radius = agent_radius,
                ) : nothing,
                dispersion_policy = in(:PotentialFields, env.deconflict_strategies) ?
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

"""
Adjust the priority of agents to manage their interactions and avoid
potential gridlocks.
"""
function set_agent_priority!(d::DeconflictStrategy, env, node)
    @debug "set_agent_priority! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function get_agent_position(d::DeconflictStrategy, agent)
    @debug "get_agent_position not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

"""
Calculate the maximum speed of a node based on its type and volume.
"""
get_agent_max_speed(::RobotNode) = DEFAULT_MAX_SPEED
function get_agent_max_speed(node)
    rect = get_base_geom(node, HyperrectangleKey())
    vol = LazySets.volume(rect)  # speed limited by volume
    vmax = DEFAULT_MAX_SPEED
    delta_v = vol * DEFAULT_MAX_SPEED_VOLUME_FACTOR
    return max(vmax - delta_v, DEFAULT_MIN_MAX_SPEED)
end

function set_agent_max_speed!(d::DeconflictStrategy, node, speed)
    @debug "set_agent_max_speed! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function set_agent_pref_velocity!(d::DeconflictStrategy, node, desired_velocity)
    if matches_template(Union{RobotGo, TransportUnitGo}, node)
        node.desired_twist = desired_velocity
    end
end

function get_agent_pref_velocity(d::DeconflictStrategy, agent)
    if matches_template(Union{RobotGo, TransportUnitGo}, agent)
        return agent.node.desired_twist
    end
end

function set_agent_alpha!(d::DeconflictStrategy, node, alpha = 0.5)
    node.alpha = alpha
end
