using PyCall

struct IntWrapper
    idx::Int
end

const RVOAgentMap = NGraph{DiGraph,IntWrapper,AbstractID}
rvo_map_num_agents(m::RVOAgentMap) = nv(m)
function set_rvo_id_map!(m::RVOAgentMap,id::AbstractID,idx::Int)
    @assert nv(m) == idx "RVOAgentMap shows $(nv(m)) agents, but this index is $idx"
    @assert !has_vertex(m,id) "Agent with id $(id) has already been added to schedule"
    add_node!(m,IntWrapper(idx),id) 
end
set_rvo_id_map!(m::RVOAgentMap,idx::Int,id::AbstractID) = set_rvo_id_map!(m,id,idx)

rvo_get_agent_idx(id::AbstractID) = node_val(get_node(rvo_global_id_map(),id)).idx
rvo_get_agent_idx(node::SceneNode) = rvo_get_agent_idx(node_id(node))
rvo_get_agent_idx(node::ConstructionPredicate) = rvo_get_agent_idx(entity(node))

global RVO_ID_GLOBAL_MAP = RVOAgentMap()
function rvo_global_id_map()
	RVO_ID_GLOBAL_MAP
end
function set_rvo_global_id_map!(val)
	global RVO_ID_GLOBAL_MAP = val
end
rvo_map_num_agents() = rvo_map_num_agents(rvo_global_id_map())
set_rvo_id_map!(id::AbstractID,idx::Int) = set_rvo_id_map!(rvo_global_id_map(),id,idx)
function rvo_reset_agent_map!()
    global RVO_ID_GLOBAL_MAP = RVOAgentMap()
end

global RVO_PYTHON_MODULE = nothing
function rvo_python_module()
	RVO_PYTHON_MODULE
end
function set_rvo_python_module!(val)
	global RVO_PYTHON_MODULE = val
end
function rvo_new_sim(;
    dt::Float64=1/40.0,
    neighbor_dist::Float64=2.0,
    max_neighbors::Int=5,
    horizon::Float64=1.0,
    horizon_obst::Float64=1.0,
    radius::Float64=0.5,
    max_speed::Float64=rvo_default_max_speed(),
    default_vel=(0.0,0.0)
    )
    rvo_reset_agent_map!()
    rvo_python_module().PyRVOSimulator(
        dt,neighbor_dist,max_neighbors,horizon,horizon_obst,radius,max_speed,default_vel
        )
end

# """
#     RVOSimWrapper

# A wrapper so that can be shared even as the underlying rvo simulations are 
# constructed and deleted.
# """
# mutable struct RVOSimWrapper
#     sim
#     # is_up_to_date::Bool
#     # timestamp::Float64
# end
# GraphUtils.is_up_to_date(m::RVOSimWrapper) = m.is_up_to_date
# GraphUtils.time_stamp(m::RVOSimWrapper) = m.timestamp

# global RVO_SIM_WRAPPER = RVOSimWrapper(nothing)
global RVO_SIM_WRAPPER = CachedElement{Any}(nothing,false,time())
rvo_global_sim_wrapper() = RVO_SIM_WRAPPER
function rvo_set_new_sim!(sim=rvo_new_sim())
    set_element!(rvo_global_sim_wrapper(),sim)
    # rvo_global_sim_wrapper().sim = sim
end
# rvo_global_sim() = rvo_global_sim_wrapper().sim
rvo_global_sim() = get_element(rvo_global_sim_wrapper())

function rvo_get_agent_position(n)
    rvo_idx = rvo_get_agent_idx(n)
    rvo_global_sim().getAgentPosition(rvo_idx)
end

# RVO PARAMETERS
global RVO_MAX_SPEED_VOLUME_FACTOR     = 0.1
global RVO_MAX_SPEED                   = 1.0
global RVO_MIN_MAX_SPEED               = 0.1
function rvo_default_max_speed()
	RVO_MAX_SPEED
end
function set_rvo_default_max_speed!(val)
	global RVO_MAX_SPEED = val
end
function rvo_default_max_speed_volume_factor()
	RVO_MAX_SPEED_VOLUME_FACTOR
end
function set_rvo_default_max_speed_volume_factor!(val)
	global RVO_MAX_SPEED_VOLUME_FACTOR = val
end
function rvo_default_min_max_speed()
	RVO_MIN_MAX_SPEED
end
function set_rvo_default_min_max_speed!(val)
	global RVO_MIN_MAX_SPEED = val
end

""" get_rvo_max_speed(node) """
get_rvo_max_speed(::RobotNode) = rvo_default_max_speed()          
function get_rvo_max_speed(node)         
    rect = get_base_geom(node,HyperrectangleKey())
    vol = LazySets.volume(rect)
    # Speed limited by volume
    vmax = rvo_default_max_speed()
    delta_v = vol * rvo_default_max_speed_volume_factor()
    max_speed = max(vmax - delta_v, rvo_default_min_max_speed())
end

""" get_rvo_radius(node) """
get_rvo_radius(node) = get_base_geom(node,HypersphereKey()).radius


global RVO_DEFAULT_TIME_STEP = 1/40.0
function rvo_default_time_step()
    RVO_DEFAULT_TIME_STEP
end 
function set_rvo_default_time_step!(val)
    global RVO_DEFAULT_TIME_STEP = val
end 


global RVO_DEFAULT_NEIGHBOR_DISTANCE = 2.0
global RVO_DEFAULT_MIN_NEIGHBOR_DISTANCE = 1.0
global RVO_DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR = 1.0
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
function rvo_default_neighborhood_velocity_scale_factor()
	RVO_DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR
end
function set_rvo_default_neighborhood_velocity_scale_factor!(val)
	global RVO_DEFAULT_NEIGHBORHOOD_VELOCITY_SCALE_FACTOR = val
end 

function get_rvo_neighbor_distance(node)
    d = rvo_default_neighbor_distance()
    v_ratio = get_rvo_max_speed(node) / rvo_default_max_speed()
    delta_d = v_ratio * rvo_default_neighborhood_velocity_scale_factor()
    d = max(d - delta_d, rvo_default_min_neighbor_distance())
end

function rvo_add_agent!(agent::Union{RobotNode,TransportUnitNode},sim)
    rad             = get_rvo_radius(agent)
    max_speed       = get_rvo_max_speed(agent)
    neighbor_dist   = get_rvo_neighbor_distance(agent)
    pt = HG.project_to_2d(global_transform(agent).translation)
    agent_idx = sim.addAgent((pt[1],pt[2]))
    set_rvo_id_map!(node_id(agent),agent_idx)
    sim.setAgentNeighborDist(agent_idx, neighbor_dist)
    sim.setAgentMaxSpeed(agent_idx,     max_speed)
    sim.setAgentRadius(agent_idx,       rad)
    return agent_idx
end
# function rvo_remove_agent!(id,sim)
#     agent_idx = get_node(rvo_global_id_map(),id).idx
#     rem_node!(rvo_global_id_map(),id)
# end
function rvo_set_agent_pref_velocity!(node,vel)
    idx = rvo_get_agent_idx(node)
    rvo_global_sim().setAgentPrefVelocity(idx,(vel[1],vel[2]))
end

for T in (
    :RobotStart,
    :RobotGo,
    :FormTransportUnit,
    :TransportUnitGo,
    :DepositCargo
)
    @eval begin
        rvo_eligible_node(n::$T) = true
    end
end
rvo_eligible_node(n) = false
rvo_eligible_node(n::ScheduleNode) = rvo_eligible_node(n.node)
rvo_eligible_node(n::CustomNode) = rvo_eligible_node(n.node)

function rvo_add_agents!(scene_tree,active_nodes,sim=rvo_global_sim())
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode,node)
            if is_root_node(scene_tree,node)
                idx = rvo_add_agent!(node,sim)
            end
        elseif matches_template(TransportUnitNode,node)
            if is_in_formation(node,scene_tree)
                idx = rvo_add_agent!(node,sim)
            end
        end
    end
    # for node in active_nodes
    #     if rvo_eligible_node(node)
    #         idx = rvo_add_agent!(entity(node),sim)
    #     end
    # end
end

function rvo_sim_needs_update(scene_tree,active_nodes)
    for node in get_nodes(scene_tree)
        if matches_template(RobotNode,node)
            if is_root_node(scene_tree,node)
                if !has_vertex(rvo_global_id_map(),node_id(node))
                    return true
                end
            end
        elseif matches_template(TransportUnitNode,node)
            if is_in_formation(node,scene_tree)
                if !has_vertex(rvo_global_id_map(),node_id(node))
                    return true
                end
            end
        end
    end
    return false
    # ids = Set{AbstractID}([node_id(n) for n in active_nodes])
    # if length(ids) == length(intersect(ids,get_vtx_ids(rvo_global_id_map())))
    #     return false
    # else
    #     return true
    # end
end

function rvo_update_sim!(active_nodes=get_nodes(scene_tree))
    if rvo_sim_needs_update(active_nodes)
        rvo_set_new_sim!()
        rvo_add_agents!(rvo_global_sim(),active_nodes)
        @info "Initializing new global rvo simulator"
    end
    @info "No need to initialize new global rvo simulator"
    rvo_global_sim()
end