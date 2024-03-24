abstract type DeconflictStrategy end

include("reciprocal_velocity_obstacle.jl")
include("tangent_bug_policy.jl")
include("potential_fields.jl")
include("combined_rvo_policy.jl")
include("custom_policy.jl")

@with_kw mutable struct NoDeconfliction <: DeconflictStrategy
    name::String = "NoDeconfliction"
end

# Set deconfliction_strategy parameter using key, e.g. :RVO, :Nothing, etc.
const supported_deconfliction_options = Dict(
    :RVO => ReciprocalVelocityObstacle(),
    :TangentBugPolicy => TangentBugPolicy(),
    :PotentialFields => PotentialFields(),
    :CombinedRVOPolicy => CombinedRVOPolicy(),
    :Nothing => NoDeconfliction(),
    # Add new deconfliction strategies here
)

# Interface methods (called by default when deconfliction_strategy is
# unrecognized, unsupported, or :Nothing)
"""
Computes twist for an agent given its goal position, with no deconfliction
strategy (robots will be seen going through each other). 
"""
function perform_twist_deconfliction(d::DeconflictStrategy, env, node)
    @unpack dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    return compute_twist_from_goal(env, agent, goal, dt)
end

function update_env_with_deconfliction(d::DeconflictStrategy, scene_tree, env)
    @debug "update_env_with_deconfliction not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function update_simulation!(d::DeconflictStrategy, env)
    @debug "update_simulation! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function update_agent_position_in_sim!(d::DeconflictStrategy, env, agent)
    @debug "update_agent_position_in_sim! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

"""
Adjust the priority of agents to manage their interactions and avoid
potential gridlocks.
"""
function set_agent_priority!(d::DeconflictStrategy, env, agent)
    @debug "set_agent_priority! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

"""
"""
function set_agent_properties(d::DeconflictStrategy)
    @debug "set_agent_properties not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

"""
Swap positions of two robots in simulation.
"""
function swap_positions!(d::DeconflictStrategy, agent1, agent2)
    @info "Swapping agent $(summary(node_id(agent1))) with 
    $(summary(node_id(agent2)))"
    tmp = global_transform(agent1)
    set_desired_global_transform!(agent1, global_transform(agent2))
    set_desired_global_transform!(agent2, tmp)
    return agent1, agent2
end

function get_agent_position(d::DeconflictStrategy, agent)
    @debug "get_agent_position not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function set_agent_position!(d::DeconflictStrategy, agent, pos)
    @debug "set_agent_position! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

"""
Calculate the maximum speed of a node based on its type and volume.
"""
get_agent_max_speed(::RobotNode) = DEFAULT_MAX_SPEED
function get_agent_max_speed(agent)
    rect = get_base_geom(agent, HyperrectangleKey())
    vol = LazySets.volume(rect)  # speed limited by volume
    vmax = DEFAULT_MAX_SPEED
    delta_v = vol * DEFAULT_MAX_SPEED_VOLUME_FACTOR
    return max(vmax - delta_v, DEFAULT_MIN_MAX_SPEED)
end

function set_agent_max_speed!(d::DeconflictStrategy, agent, speed)
    @debug "set_agent_max_speed! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
end

function set_agent_pref_velocity!(d::DeconflictStrategy, agent, desired_velocity)
    if matches_template(Union{RobotGo, TransportUnitGo}, agent)
        agent.desired_twist = desired_velocity
    end
end

function get_agent_pref_velocity(d::DeconflictStrategy, agent)
    if matches_template(Union{RobotGo, TransportUnitGo}, agent)
        return agent.node.desired_twist
    end
end

function get_agent_alpha(d::DeconflictStrategy, agent)
    @debug "get_agent_alpha! not implemented for deconfliction type: 
    $(join(env.d.name, ", "))"
    if matches_template(Union{RobotGo, TransportUnitGo}, agent)
        return agent.alpha
    end
end

function set_agent_alpha!(d::DeconflictStrategy, agent, alpha = 0.5)
    agent.alpha = alpha
end
