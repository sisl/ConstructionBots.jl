using PyCall

# Note: A dispersion policy is recommended when using the 
# ReciprocalVelocityObstacle policy.
@with_kw mutable struct ReciprocalVelocityObstacle <: DeconflictStrategy
    name::String = "ReciprocalVelocityObstacle"
    dt::Float64 = 1/40.0
    max_speed::Float64 = DEFAULT_MAX_SPEED
    max_speed_volume_factor::Float64 = DEFAULT_MAX_SPEED_VOLUME_FACTOR
    min_max_speed::Float64 = DEFAULT_MIN_MAX_SPEED
    default_time_step::Float64 = DEFAULT_TIME_STEP
    neighbor_distance::Float64 = DEFAULT_NEIGHBOR_DISTANCE
    min_neighbor_distance::Float64 = DEFAULT_MINIMUM_NEIGHBOR_DISTANCE
    max_neighbors::Int = 5
    horizon::Float64 = 2.0
    horizon_obst::Float64 = 1.0
    default_radius::Float64 = 0.5
    default_velocity::Tuple{Float64, Float64} = (0.0, 0.0)
end

"""
    perform_twist_deconfliction(r::ReciprocalVelocityObstacle, env, node)

Calculates the optimal twist (linear and angular velocity) to avoid collisions 
for a given agent using the Reciprocal Velocity Obstacle method. The method 
adjusts the agent's path to avoid potential collisions with other agents and 
obstacles while trying to reach its goal.

# Arguments
- `r`: The ReciprocalVelocityObstacle deconfliction strategy instance that
contains necessary parameters.
- `env`: The simulation environment with current state of agents and obstacles.
- `node`: The agent for which the twist is being calculated.

# Returns
- `Twist`: The desired twist (comprising linear and angular velocity) that
avoids collisions and progresses agent towards its goal.
"""
function perform_twist_deconfliction(r::ReciprocalVelocityObstacle, env, node)
    @unpack sched, scene_tree, agent_policies, dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    mode = :not_set
    twist = compute_twist_from_goal(env, agent, goal, dt)
    # Set nominal velocity to zero if close to goal (Note: this is a hack)
    parent_step = get_parent_build_step(sched, node)
    if !(parent_step === nothing)
        countdown = active_build_step_countdown(parent_step.node, env)
        dist_to_goal = norm(goal.translation .- global_transform(agent).translation)
        unit_radius = get_base_geom(entity(node), HypersphereKey()).radius
        if (mode != :EXIT_CIRCLE) && (dist_to_goal < 15 * unit_radius)
            # Done so the agent doesn't crowd its destination
            if matches_template(TransportUnitGo, node) && countdown >= 1
                twist = Twist(0.0 * twist.vel, twist.ω)
            elseif matches_template(RobotGo, node) && countdown >= 3
                twist = Twist(0.0 * twist.vel, twist.ω)
            end
        end
    end
    return twist
end

struct IntWrapper idx::Int end

const RVOAgentMap = NGraph{DiGraph,IntWrapper,AbstractID}

rvo_map_num_agents(m::RVOAgentMap) = nv(m)

function set_rvo_id_map!(m::RVOAgentMap, id::AbstractID, idx::Int)
    @assert nv(m) == idx "RVOAgentMap shows $(nv(m)) agents, but this index is $idx"
    @assert !has_vertex(m, id) "Agent with id $(id) has already been added to schedule"
    add_node!(m, IntWrapper(idx), id)
end

set_rvo_id_map!(m::RVOAgentMap, idx::Int, id::AbstractID) = set_rvo_id_map!(m, id, idx)
rvo_get_agent_idx(id::AbstractID) = node_val(get_node(rvo_global_id_map(), id)).idx
rvo_get_agent_idx(node::SceneNode) = rvo_get_agent_idx(node_id(node))
rvo_get_agent_idx(node::ConstructionPredicate) = rvo_get_agent_idx(entity(node))
rvo_get_agent_idx(node::ScheduleNode) = rvo_get_agent_idx(node.node)

global RVO_ID_GLOBAL_MAP = RVOAgentMap()

function rvo_global_id_map()
    RVO_ID_GLOBAL_MAP
end

function set_rvo_global_id_map!(val)
    global RVO_ID_GLOBAL_MAP = val
end

rvo_map_num_agents() = rvo_map_num_agents(rvo_global_id_map())
set_rvo_id_map!(id::AbstractID, idx::Int) = set_rvo_id_map!(rvo_global_id_map(), id, idx)

function rvo_reset_agent_map!()
    global RVO_ID_GLOBAL_MAP = RVOAgentMap()
end

rvo_active_agents(scene_tree) =
    (get_node(scene_tree, node_id(n)) for n in get_nodes(rvo_global_id_map()))

function rvo_new_sim(env)
    rvo_reset_agent_map!()
    rvo_instance = env.deconfliction_type
    dt=rvo_instance.dt
    neighbor_dist=rvo_instance.neighbor_distance
    max_neighbors=rvo_instance.max_neighbors
    horizon=rvo_instance.horizon
    horizon_obst=rvo_instance.horizon_obst
    radius=rvo_instance.default_radius
    max_speed=rvo_instance.max_speed
    default_vel=rvo_instance.default_velocity
    rvo_python_module = pyimport("rvo2")

    rvo_python_module.PyRVOSimulator(
        dt,
        neighbor_dist,
        max_neighbors,
        horizon,
        horizon_obst,
        radius,
        max_speed,
        default_vel,
    )
end

global RVO_SIM_WRAPPER = CachedElement{Any}(nothing, false, time())
rvo_global_sim_wrapper() = RVO_SIM_WRAPPER

function rvo_set_new_sim!(env, sim = rvo_new_sim(env))
    set_element!(rvo_global_sim_wrapper(), sim)
end

rvo_global_sim() = get_element(rvo_global_sim_wrapper())

""" get_rvo_radius(node) """
get_rvo_radius(node) = get_base_geom(node, HypersphereKey()).radius

function set_rvo_default_neighbor_distance!(val)
    DEFAULT_NEIGHBOR_DISTANCE = val
end

function set_rvo_default_min_neighbor_distance!(val)
    RVO_DEFAULT_MIN_NEIGHBOR_DISTANCE = val
end

function get_rvo_neighbor_distance(node)
    d = DEFAULT_NEIGHBOR_DISTANCE
    v_ratio = get_agent_max_speed(node) / DEFAULT_MAX_SPEED
    delta_d = v_ratio * DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR
    d = max(d - delta_d, DEFAULT_MINIMUM_NEIGHBOR_DISTANCE)
end

function rvo_add_agent!(agent::Union{RobotNode,TransportUnitNode}, sim)
    rad = get_rvo_radius(agent) * 1.05 # Add a little bit of padding for visualization
    max_speed = get_agent_max_speed(agent)
    neighbor_dist = get_rvo_neighbor_distance(agent)
    pt = project_to_2d(global_transform(agent).translation)
    agent_idx = sim.addAgent((pt[1], pt[2]))
    set_rvo_id_map!(node_id(agent), agent_idx)
    sim.setAgentNeighborDist(agent_idx, neighbor_dist)
    sim.setAgentMaxSpeed(agent_idx, max_speed)
    sim.setAgentRadius(agent_idx, rad)
    return agent_idx
end

for T in (:RobotStart, :RobotGo, :FormTransportUnit, :TransportUnitGo, :DepositCargo)
    @eval begin
        rvo_eligible_node(n::$T) = true
    end
end
rvo_eligible_node(n) = false

function rvo_add_agents!(scene_tree, sim = rvo_global_sim())
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode, node)
            if is_root_node(scene_tree, node)
                idx = rvo_add_agent!(node, sim)
            end
        elseif matches_template(TransportUnitNode, node)
            if is_in_formation(node, scene_tree)
                idx = rvo_add_agent!(node, sim)
            end
        end
    end
end

function rvo_sim_needs_update(scene_tree)
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode, node)
            if is_root_node(scene_tree, node)
                if !has_vertex(rvo_global_id_map(), node_id(node))
                    return true
                end
            end
        elseif matches_template(TransportUnitNode, node)
            if is_in_formation(node, scene_tree)
                if !has_vertex(rvo_global_id_map(), node_id(node))
                    return true
                end
            end
        end
    end
    return false
end

function update_rvo_sim!(env)
    @unpack sched, scene_tree, cache = env
    active_nodes = [get_node(sched, v) for v in cache.active_set]
    if rvo_sim_needs_update(scene_tree)
        @info "New RVO simulation"
        rvo_set_new_sim!(env)
        rvo_add_agents!(scene_tree)
        for node in active_nodes
            set_rvo_priority!(env, node)
        end
    end
end

function update_env_with_deconfliction(r::ReciprocalVelocityObstacle, scene_tree, env)
    rvo_set_new_sim!(env)
    rvo_add_agents!(scene_tree)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_agent_max_speed(n)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = nothing,
                dispersion_policy = nothing,
            )
        end
    end
end

function update_simulation!(r::ReciprocalVelocityObstacle, env)
    update_rvo_sim!(env)
end

function update_agent_position_in_sim!(r::ReciprocalVelocityObstacle, env, agent)
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

# Ideally, we want to set the priority of agents only within a certain region. If we
# can do this dynamically, we could also set one agent to have an α of 0, which would help
# reduce grid lock (by forcing other agents to move around it)
"""
    set_rvo_priority!(env, node)

Low alpha means higher priority
"""
function set_rvo_priority!(env, node) end

function set_rvo_priority!(
    env,
    node::Union{RobotStart,RobotGo,FormTransportUnit,TransportUnitGo,DepositCargo},
)
    if matches_template(Union{FormTransportUnit,DepositCargo}, node)
        alpha = 0.0
    elseif matches_template(TransportUnitGo, node)
        if parent_build_step_is_active(node, env)
            c_id = cargo_id(entity(node)).id
            alpha = 0.0 + c_id / (10 * env.max_cargo_id)
        else
            alpha = 1.0
        end
    elseif parent_build_step_is_active(node, env)
        if cargo_ready_for_pickup(node, env)
            alpha = 0.1
        else
            alpha = 0.5
        end
    else
        alpha = 1.0
    end
    set_agent_alpha!(env.deconfliction_type, node, alpha)
end

function set_agent_properties(r::ReciprocalVelocityObstacle)
    set_rvo_default_neighbor_distance!(16 * default_robot_radius())
    set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())
end

function set_agent_priority!(r::ReciprocalVelocityObstacle, env, agent)
    return set_rvo_priority!(env, agent)
end

function get_agent_position(r::ReciprocalVelocityObstacle, agent)
    rvo_idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentPosition(rvo_idx)
end

function set_agent_position!(r::ReciprocalVelocityObstacle, agent, pos)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPosition(idx, (pos[1], pos[2]))
end

function set_agent_max_speed!(r::ReciprocalVelocityObstacle, agent, speed=get_agent_max_speed(agent))
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentMaxSpeed(idx, speed)
end

function get_agent_velocity(r::ReciprocalVelocityObstacle, agent)
    idx = rvo_get_agent_idx(agent)
    rvo_global_sim().getAgentVelocity(idx)
end

function get_agent_pref_velocity(r::ReciprocalVelocityObstacle, agent)
    idx = rvo_get_agent_idx(entity(agent))
    return rvo_global_sim().getAgentPrefVelocity(idx)
end

function set_agent_pref_velocity!(r::ReciprocalVelocityObstacle, agent, desired_velocity)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentPrefVelocity(idx, (desired_velocity[1], desired_velocity[2]))
end

function get_agent_alpha(r::ReciprocalVelocityObstacle, agent)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().getAgentAlpha(idx)
end

function set_agent_alpha!(r::ReciprocalVelocityObstacle, agent, alpha=0.5)
    idx = rvo_get_agent_idx(agent)
    return rvo_global_sim().setAgentAlpha(idx, alpha)
end