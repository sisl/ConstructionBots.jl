using PyCall

@with_kw mutable struct ReciprocalVelocityObstacle <: DeconflictStrategy
    name::String="ReciprocalVelocityObstacle"
    # TODO(tashakim): store relevant fields
end

function perform_twist_deconfliction(ReciprocalVelocityObstacle, params)
    # TODO(tashakim): implement this method
end

struct IntWrapper idx::Int end

function ensure_rvo_python_module_loaded!()
    if rvo_python_module() === nothing
        reset_rvo_python_module!()
    end
end

const RVOAgentMap = NGraph{DiGraph,IntWrapper,AbstractID}

rvo_map_num_agents(m::RVOAgentMap) = nv(m)

function set_rvo_id_map!(m::RVOAgentMap, id::AbstractID, idx::Int)
    ensure_rvo_python_module_loaded!()
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

global RVO_PYTHON_MODULE = nothing

function rvo_python_module()
    RVO_PYTHON_MODULE
end

function set_rvo_python_module!(val)
    global RVO_PYTHON_MODULE = val
end

function reset_rvo_python_module!()
    set_rvo_python_module!(nothing)
    set_rvo_python_module!(pyimport("rvo2"))
end

function rvo_new_sim(;
    dt::Float64 = 1 / 40.0,
    # TODO(tashakim): Store neighbor_dist, max_neighbors, horizon, 
    # horizon_obst, radius as fields in RVO state
    neighbor_dist::Float64 = 2.0,
    max_neighbors::Int = 5,
    horizon::Float64 = 2.0,
    horizon_obst::Float64 = 1.0,
    radius::Float64 = 0.5,
    max_speed::Float64 = DEFAULT_MAX_SPEED,
    default_vel = (0.0, 0.0),
)
    reset_rvo_python_module!()
    rvo_reset_agent_map!()
    rvo_python_module().PyRVOSimulator(
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

function rvo_set_new_sim!(sim = rvo_new_sim())
    ensure_rvo_python_module_loaded!()
    set_element!(rvo_global_sim_wrapper(), sim)
end

rvo_global_sim() = get_element(rvo_global_sim_wrapper())

# TODO(tashakim): Remove this method after implementing 
# get_agent_max_speed method
""" get_rvo_max_speed(node) """
get_rvo_max_speed(::RobotNode) = DEFAULT_MAX_SPEED
function get_rvo_max_speed(node)
    ensure_rvo_python_module_loaded!()
    rect = get_base_geom(node, HyperrectangleKey())
    vol = LazySets.volume(rect)
    # Speed limited by volume
    vmax = DEFAULT_MAX_SPEED
    delta_v = vol * DEFAULT_MAX_SPEED_VOLUME_FACTOR
    return max(vmax - delta_v, DEFAULT_MIN_MAX_SPEED)
end

""" get_rvo_radius(node) """
get_rvo_radius(node) = get_base_geom(node, HypersphereKey()).radius

global RVO_DEFAULT_NEIGHBOR_DISTANCE = 2.0
global RVO_DEFAULT_MIN_NEIGHBOR_DISTANCE = 1.0

function rvo_default_neighbor_distance()
    RVO_DEFAULT_NEIGHBOR_DISTANCE
end

function set_rvo_default_neighbor_distance!(val)
    global RVO_DEFAULT_NEIGHBOR_DISTANCE = val
end

function rvo_default_min_neighbor_distance()
    RVO_DEFAULT_MIN_NEIGHBOR_DISTANCE
end

function set_rvo_default_min_neighbor_distance!(val)
    global RVO_DEFAULT_MIN_NEIGHBOR_DISTANCE = val
end

function get_rvo_neighbor_distance(node)
    ensure_rvo_python_module_loaded!()
    d = rvo_default_neighbor_distance()
    v_ratio = get_rvo_max_speed(node) / DEFAULT_MAX_SPEED
    delta_d = v_ratio * DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR
    d = max(d - delta_d, rvo_default_min_neighbor_distance())
end

function rvo_add_agent!(agent::Union{RobotNode,TransportUnitNode}, sim)
    ensure_rvo_python_module_loaded!()
    rad = get_rvo_radius(agent) * 1.05 # Add a little bit of padding for visualization
    max_speed = get_rvo_max_speed(agent)
    neighbor_dist = get_rvo_neighbor_distance(agent)
    pt = project_to_2d(global_transform(agent).translation)
    agent_idx = sim.addAgent((pt[1], pt[2]))
    set_rvo_id_map!(node_id(agent), agent_idx)
    sim.setAgentNeighborDist(agent_idx, neighbor_dist)
    sim.setAgentMaxSpeed(agent_idx, max_speed)
    sim.setAgentRadius(agent_idx, rad)
    return agent_idx
end

function rvo_get_agent_position(n)
    rvo_idx = rvo_get_agent_idx(n)
    rvo_global_sim().getAgentPosition(rvo_idx)
end

function rvo_set_agent_position!(node, pos)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().setAgentPosition(idx, (pos[1], pos[2]))
end

function rvo_set_agent_pref_velocity!(node, vel)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().setAgentPrefVelocity(idx, (vel[1], vel[2]))
end

function rvo_get_agent_pref_velocity(node)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().getAgentPrefVelocity(idx)
end
function rvo_get_agent_velocity(node)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().getAgentVelocity(idx)
end

function rvo_set_agent_max_speed!(node, speed = get_rvo_max_speed(node))
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().setAgentMaxSpeed(idx, speed)
end

function rvo_set_agent_alpha!(node, alpha = 0.5)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().setAgentAlpha(idx, alpha)
end

function rvo_get_agent_alpha(node)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().getAgentAlpha(idx)
end

for T in (:RobotStart, :RobotGo, :FormTransportUnit, :TransportUnitGo, :DepositCargo)
    @eval begin
        rvo_eligible_node(n::$T) = true
    end
end
rvo_eligible_node(n) = false

function rvo_add_agents!(scene_tree, sim = rvo_global_sim())
    ensure_rvo_python_module_loaded!()
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
    ensure_rvo_python_module_loaded!()
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
        rvo_set_new_sim!()
        rvo_add_agents!(scene_tree)
        for node in active_nodes
            set_rvo_priority!(env, node)
        end
    end
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
    set_agent_alpha!(env, node, alpha)
end
