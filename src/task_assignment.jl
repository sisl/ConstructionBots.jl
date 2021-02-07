TaskGraphs.duration_lower_bound(node::ConstructionPredicate) = 0.0
function TaskGraphs.duration_lower_bound(node::ConstructionPredicate,start,goal)
    d = norm(goal-start)
    max_speed = get_rvo_max_speed(entity(node))
    dt = d/max_speed
end
function TaskGraphs.duration_lower_bound(node::EntityGo)
    start = global_transform(start_config(node)).translation
    goal = global_transform(goal_config(node)).translation
    TaskGraphs.duration_lower_bound(node,start,goal)
end
function TaskGraphs.duration_lower_bound(node::Union{FormTransportUnit,DepositCargo})
    start = global_transform(cargo_start_config(node)).translation
    goal = global_transform(cargo_goal_config(node)).translation
    TaskGraphs.duration_lower_bound(node,start,goal)
end

TaskGraphs.is_tight(node::Union{TransportUnitGo,RobotGo}) = true

TaskGraphs.needs_path(node::BuildPhasePredicate)    = false
TaskGraphs.needs_path(node::ObjectStart)            = false
TaskGraphs.needs_path(node::AssemblyComplete)       = false
TaskGraphs.needs_path(node::AssemblyStart)          = false
TaskGraphs.needs_path(node::ProjectComplete)        = false

function TaskGraphs.generate_path_spec(node::ConstructionPredicate)
    PathSpec(
        min_duration=TaskGraphs.duration_lower_bound(node),
        tight=TaskGraphs.is_tight(node),
        static=TaskGraphs.is_static(node),
        free=TaskGraphs.is_free(node),
        plan_path=TaskGraphs.needs_path(node)
    )
end
TaskGraphs.generate_path_spec(::SceneTree,node) = generate_path_spec(node)
TaskGraphs.generate_path_spec(::OperatingSchedule,scene_tree::SceneTree,node) = generate_path_spec(scene_tree,node)

function convert_to_operating_schedule(sched)
    tg_sched = OperatingSchedule()
    for node in get_nodes(sched)
        add_node!(tg_sched,ScheduleNode(node_id(node),node_val(node),
            generate_path_spec(node_val(node))))
        if matches_template(ProjectComplete,node)
            push!(tg_sched.terminal_vtxs, get_vtx(tg_sched, node_id(node)))
        end
    end
    for e in edges(sched)
        add_edge!(tg_sched,get_vtx_id(sched,e.src),get_vtx_id(sched,e.dst))
    end
    tg_sched
end

# CRCBS.is_valid(node::ConstructionPredicate) = CRCBS.is_valid(node_id(node))
CRCBS.is_valid(node::SceneNode) = CRCBS.is_valid(node_id(node))

TaskGraphs.align_with_successor(node::ConstructionPredicate,succ::ConstructionPredicate) = node
TaskGraphs.align_with_successor(node::RobotGo,succ::T) where {T<:Union{EntityConfigPredicate,EntityGo}} = RobotGo(
    TaskGraphs.first_valid(entity(node),entity(succ)),
    start_config(node),
    start_config(succ),
    node_id(node)
    )
TaskGraphs.align_with_predecessor(node::RobotGo,pred::T) where {T<:Union{EntityConfigPredicate,EntityGo}} = RobotGo(
    TaskGraphs.first_valid(entity(node),entity(pred)),
    goal_config(pred),
    goal_config(node),
    node_id(node)
    )
# TaskGraphs.align_with_successor(node::T,succ::S) where {C,T<:EntityConfigPredicate{C},S<:EntityGo{C}} = T(first_valid(node,succ),start_config(node))

ALIGNMENT_CHECK_TOLERANCE = 1e-3

function get_matching_child_id(node::Union{FormTransportUnit,DepositCargo},pred::RobotGo)
    t_rel = HG.relative_transform(
        global_transform(goal_config(node)),
        global_transform(goal_config(pred))
        )
    transport_unit = entity(node)
    for id in collect(keys(robot_team(transport_unit)))
        tform = child_transform(transport_unit,id)
        if norm(tform.translation - t_rel.translation) <= 1e-3
            return id
		end
    end
    return nothing
end

function TaskGraphs.align_with_predecessor(node::FormTransportUnit,pred::RobotGo)
    matching_id = get_matching_child_id(node,pred)
    if !(matching_id === nothing)
        transport_unit = entity(node)
        HG.swap_robot_id!(transport_unit,matching_id,node_id(entity(pred)))
    end
	node
end
function TaskGraphs.align_with_predecessor(node::RobotGo,pred::DepositCargo) 
    matching_id = get_matching_child_id(pred,node)
    if !(matching_id === nothing)
        if valid_id(matching_id)
            return RobotGo(RobotNode(matching_id,entity(node)),start_config(node),goal_config(node),node_id(node))
        end
    end
    return node
end

"""
    build_and_link_dummy_robot!(node,id,tform)

Create the dummy `RobotGo` nodes to go in (if `node::FormTransportUnit`) or out
(if `node::DepositCargo`) of node.
"""
function build_and_link_dummy_robot!(node,id,tform)
    robot_go = RobotGo(RobotNode(id,GeomNode(nothing)))
    set_parent!(goal_config(robot_go),goal_config(node))
    set_parent!(start_config(robot_go),goal_config(robot_go))
    set_local_transform!(goal_config(robot_go),tform)
    robot_go
end

function add_dummy_robot_go_nodes!(sched)
    for node in get_nodes(sched)
        if matches_template(FormTransportUnit,node)
            transport_unit = entity(node)
            for (id,tform) in robot_team(transport_unit)
                robot_go = build_and_link_dummy_robot!(node,id,tform) 
                add_node!(sched, robot_go)
                @assert add_edge!(sched,robot_go,node)
            end
        elseif matches_template(DepositCargo,node)
            transport_unit = entity(node)
            for (id,tform) in robot_team(transport_unit)
                robot_go = build_and_link_dummy_robot!(node,id,tform) 
                # add_node!(sched, robot_go)
                add_node!(sched, robot_go)
                @assert add_edge!(sched,node,robot_go)
            end
        end
    end
end
