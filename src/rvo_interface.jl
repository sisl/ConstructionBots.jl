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
global RVO_SIM = nothing
function rvo_python_module()
	RVO_PYTHON_MODULE
end
function set_rvo_python_module!(val)
	global RVO_PYTHON_MODULE = val
end
function rvo_sim()
	RVO_SIM
end
function set_rvo_sim!(val)
	global RVO_SIM = val
end
function rvo_new_sim(;
    dt::Float64=1/40.0,
    neighbor_dist::Float64=2.0,
    max_neighbors::Int=5,
    horizon::Float64=1.0,
    horizon_obst::Float64=1.0,
    radius::Float64=0.5,
    max_speed::Float64=rvo_default_max_speed(),
    )
    rvo_reset_agent_map!()
    rvo_python_module().PyRVOSimulator(
        dt,neighbor_dist,max_neighbors,horizon,horizon_obst,radius,max_speed
        )
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
	RVO_MIN_SPEED
end
function set_rvo_default_min_max_speed!(val)
	global RVO_MIN_SPEED = val
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

function rvo_add_agent!(node,sim)
    rad             = get_rvo_radius(node)
    max_speed       = get_rvo_max_speed(node)
    neighbor_dist   = get_rvo_neighbor_distance(node)
    pt = HG.project_to_2d(global_transform(node).translation)
    agent_idx = sim.addAgent((pt[1],pt[2]))
    set_rvo_id_map!(node_id(node),agent_idx)
    sim.setAgentNeighborDist(agent_idx, neighbor_dist)
    sim.setAgentMaxSpeed(agent_idx,     max_speed)
    sim.setAgentRadius(agent_idx,       rad)
    return agent_idx
end
# function rvo_remove_agent!(id,sim)
#     agent_idx = get_node(rvo_global_id_map(),id).idx
#     rem_node!(rvo_global_id_map(),id)
# end

function rvo_add_agents!(scene_tree,sim,active_nodes=get_nodes(scene_tree))
    for node in active_nodes
        if matches_template(RobotNode,node)
            idx = rvo_add_agent!(node,sim)
        end
    end
end