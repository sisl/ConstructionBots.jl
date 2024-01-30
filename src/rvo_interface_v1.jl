using PyCall
using Graphs
using Logging
using Dates

# Custom type definitions
struct IntWrapper
    idx::Int
end

struct RVOAgentMap
    graph::NGraph{DiGraph,IntWrapper,AbstractID}
    RVOAgentMap() = new(NGraph{DiGraph,IntWrapper,AbstractID}())
end

struct RVO <: DeconflictStrategy
    id_map::RVOAgentMap
    python_module::Any
    sim_wrapper::CachedElement{Any}
    max_speed::Float64
    max_speed_volume_factor::Float64
    min_max_speed::Float64
    default_time_step::Float64
    default_neighbor_distance::Float64
    default_min_neighbor_distance::Float64
    default_neighborhood_velocity_scale_factor::Float64

    RVO() = new(
        RVOAgentMap(),
        nothing,
        CachedElement{Any}(nothing, false, Dates.now()),
        4.0,
        0.01,
        1.0,
        1 / 40.0,
        2.0,
        1.0,
        1.0,
    )
end

# RVO initializer and setter methods
function rvo_reset_agent_map!(state::RVO)
    state.id_map = RVOAgentMap()
end

rvo_active_agents(scene_tree) =
    (get_node(scene_tree, node_id(n)) for n in get_nodes(rvo_global_id_map()))

function reset_rvo_python_module!(state::RVO)
    state.python_module = nothing
    state.python_module = pyimport("rvo2")
end

# RVO simulation methods
function rvo_new_sim(
    state::RVO;
    dt::Float64 = 1 / 40.0,
    neighbor_dist::Float64 = 2.0,
    max_neighbors::Int = 5,
    horizon::Float64 = 2.0,
    horizon_obst::Float64 = 1.0,
    radius::Float64 = 0.5,
    max_speed::Float64 = state.max_speed,
    default_vel = (0.0, 0.0),
)
    reset_rvo_python_module!(state)
    rvo_reset_agent_map!(state)
    state.python_module.PyRVOSimulator(
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

function rvo_set_new_sim!(state::RVO, sim = nothing)
    sim = isnothing(sim) ? rvo_new_sim(state) : sim
    state.sim_wrapper = CachedElement{Any}(sim, true, Dates.now())
end

# RVOAgent map methods
function rvo_map_num_agents(m::RVOAgentMap)
    nv(m.graph)
end

function set_rvo_id_map!(m::RVOAgentMap, id::AbstractID, idx::Int)
    @assert nv(m.graph) == idx "RVOAgentMap shows $(nv(m.graph)) agents, but this index is $idx"
    @assert !has_vertex(m.graph, id) "Agent with id $(id) has already been added to schedule"
    add_node!(m.graph, IntWrapper(idx), id)
end

set_rvo_id_map!(m::RVOAgentMap, id::AbstractID, idx::Int) = set_rvo_id_map!(m, id, idx)

# RVOAgent methods
function rvo_add_agent!(state::RVO, agent::Union{RobotNode,TransportUnitNode}, sim)
    rad = get_rvo_radius(agent) * 1.05 # Add a little bit of padding for visualization
    max_speed = get_rvo_max_speed(state, agent)
    neighbor_dist = get_rvo_neighbor_distance(state, agent)
    pt = project_to_2d(global_transform(agent).translation)
    agent_idx = sim.addAgent((pt[1], pt[2]))
    set_rvo_id_map!(state.id_map, node_id(agent), agent_idx)
    sim.setAgentNeighborDist(agent_idx, neighbor_dist)
    sim.setAgentMaxSpeed(agent_idx, max_speed)
    sim.setAgentRadius(agent_idx, rad)
    return agent_idx
end

function rvo_get_agent_position(state::RVO, n)
    rvo_idx = rvo_get_agent_idx(state, n)
    get_element(state.sim_wrapper).getAgentPosition(rvo_idx)
end

function rvo_set_agent_position!(state::RVO, node, pos)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).setAgentPosition(idx, (pos[1], pos[2]))
end

function rvo_set_agent_pref_velocity!(state::RVO, node, vel)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).setAgentPrefVelocity(idx, (vel[1], vel[2]))
end

function rvo_get_agent_pref_velocity(state::RVO, node)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).getAgentPrefVelocity(idx)
end

function rvo_get_agent_velocity(state::RVO, node)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).getAgentVelocity(idx)
end

function rvo_set_agent_max_speed!(state::RVO, node, speed = nothing)
    speed = isnothing(speed) ? rvo_default_max_speed(state) : speed
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).setAgentMaxSpeed(idx, speed)
end

function rvo_set_agent_alpha!(state::RVO, node, alpha = 0.5)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).setAgentAlpha(idx, alpha)
end

function rvo_get_agent_alpha(state::RVO, node)
    idx = rvo_get_agent_idx(state, node)
    get_element(state.sim_wrapper).getAgentAlpha(idx)
end

# RVO parameter methods
function rvo_default_max_speed(state::RVO)
    state.max_speed
end

function set_rvo_default_max_speed!(state::RVO, val)
    state.max_speed = val
end

function rvo_default_max_speed_volume_factor(state::RVO)
    state.max_speed_volume_factor
end

function set_rvo_default_max_speed_volume_factor!(state::RVO, val)
    state.max_speed_volume_factor = val
end

function rvo_default_min_max_speed(state::RVO)
    state.min_max_speed
end

function set_rvo_default_min_max_speed!(state::RVO, val)
    state.min_max_speed = val
end

function rvo_default_time_step(state::RVO)
    state.default_time_step
end

function set_rvo_default_time_step!(state::RVO, val)
    state.default_time_step = val
end

function rvo_default_neighbor_distance(state::RVO)
    state.default_neighbor_distance
end

function set_rvo_default_neighbor_distance!(state::RVO, val)
    state.default_neighbor_distance = val
end

function rvo_default_min_neighbor_distance(state::RVO)
    state.default_min_neighbor_distance
end

function set_rvo_default_min_neighbor_distance!(state::RVO, val)
    state.default_min_neighbor_distance = val
end

function rvo_default_neighborhood_velocity_scale_factor(state::RVO)
    state.default_neighborhood_velocity_scale_factor
end

function set_rvo_default_neighborhood_velocity_scale_factor!(state::RVO, val)
    state.default_neighborhood_velocity_scale_factor = val
end

# Node-specific helper methods
"""
Calculate the maximum speed of a RobotNode defined by `rvo_default_max_speed`.
"""
get_rvo_max_speed(::RobotNode) = rvo_default_max_speed()

"""
Calculate the maximum speed of a node based on its geometry and volume.
"""
function get_rvo_max_speed(node)
    rect = get_base_geom(node, HyperrectangleKey())
    vol = LazySets.volume(rect)
    # Speed limited by volume
    vmax = rvo_default_max_speed()
    delta_v = vol * rvo_default_max_speed_volume_factor()
    return max(vmax - delta_v, rvo_default_min_max_speed())
end

"""
Calculate the radius of a node.
"""
get_rvo_radius(node) = get_base_geom(node, HypersphereKey()).radius

"""
Calculate the preferred neighbor distance for a node in the RVO simulation.

The calculation is based on default neighbor distance, adjusted using node's
maximum speed and velocity scale factor, to avoid overly close interactions.
"""
function get_rvo_neighbor_distance(state::RVO, node)
    d = rvo_default_neighbor_distance(state)
    v_ratio = get_rvo_max_speed(state, node) / rvo_default_max_speed(state)
    delta_d = v_ratio * rvo_default_neighborhood_velocity_scale_factor(state)
    max(d - delta_d, rvo_default_min_neighbor_distance(state))
end

# RVO eligibility helper methods
for T in (:RobotStart, :RobotGo, :FormTransportUnit, :TransportUnitGo, :DepositCargo)
    @eval begin
        rvo_eligible_node(n::$T) = true
    end
end

rvo_eligible_node(n) = false

function is_rvo_eligible(node)
    matches_template(RobotNode, node) && is_root_node(scene_tree, node) ||
        matches_template(TransportUnitNode, node) && is_in_formation(node, scene_tree)
end

function not_already_mapped(state::RVO, node)
    !has_vertex(state.id_map.graph, node_id(node))
end

# Agent addition and simulation update
function rvo_add_agents!(state::RVO, scene_tree)
    sim = get_element(state.sim_wrapper)
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode, node) && is_root_node(scene_tree, node)
            rvo_add_agent!(state, node, sim)
        elseif matches_template(TransportUnitNode, node) &&
               is_in_formation(node, scene_tree)
            rvo_add_agent!(state, node, sim)
        end
    end
end

function rvo_sim_needs_update(state::RVO, scene_tree)
    for node in get_nodes(scene_tree)
        if is_rvo_eligible(node) && not_already_mapped(state, node)
            return true
        end
    end
    return false
end

# Global RVO simulation wrapper
global RVO_SIM_WRAPPER = CachedElement{Any}(nothing, false, Dates.now())
rvo_global_sim_wrapper() = RVO_SIM_WRAPPER

function rvo_set_new_global_sim!(sim = nothing)
    sim = isnothing(sim) ? rvo_new_sim() : sim
    set_element!(rvo_global_sim_wrapper(), sim)
end

rvo_global_sim() = get_element(rvo_global_sim_wrapper())
