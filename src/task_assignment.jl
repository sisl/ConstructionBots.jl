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
function convert_from_operating_schedule(::Type{T},sched::OperatingSchedule) where {T}
    G = T()
    for node in get_nodes(sched)
        add_node!(G,node.node,node_id(node))
    end
    for e in edges(sched)
        add_edge!(G,get_vtx_id(sched,e.src),get_vtx_id(sched,e.dst))
    end
    G
end

# CRCBS.is_valid(node::ConstructionPredicate) = CRCBS.is_valid(node_id(node))
CRCBS.is_valid(node::SceneNode) = CRCBS.is_valid(node_id(node))

TaskGraphs.align_with_successor(node::ConstructionPredicate,succ::ConstructionPredicate) = node
TaskGraphs.align_with_successor(node::RobotGo,succ::T) where {T<:RobotGo} = RobotGo(
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

function get_matching_child_id(node::FormTransportUnit,pred::RobotGo)
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

function get_matching_child_id(node::DepositCargo,succ::RobotGo)
    t_rel = HG.relative_transform(
        global_transform(goal_config(node)),
        global_transform(start_config(succ))
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
function TaskGraphs.align_with_predecessor(sched::OperatingSchedule,node::RobotGo,pred::DepositCargo) 
    matching_id = get_matching_child_id(pred,node)
    if !(matching_id === nothing)
        if valid_id(matching_id)
            # Have to pull the RobotNode from sched, because it's not accessible through DepositCargo
            robot_start = get_node(sched,RobotStart(RobotNode(matching_id,entity(node))))
            return RobotGo(RobotNode(matching_id,entity(robot_start)),start_config(node),goal_config(node),node_id(node))
        end
    else
        @warn "$(node_id(node)) should be a child of $(node_id(pred))" node pred
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

function compute_backward_depth(sched)
    forward_depth = zeros(Int,nv(sched))
    for v in topological_sort_by_dfs(sched)
        forward_depth[v] = maximum([0, 
            map(vp->forward_depth[vp]+1,inneighbors(sched,v))...])
    end
    backward_depth = deepcopy(forward_depth)
    for v in reverse(topological_sort_by_dfs(sched))
        backward_depth[v] = maximum(
            [backward_depth[v], 
            map(vp->backward_depth[vp]-1,outneighbors(sched,v))...
            ])
    end
    return backward_depth
end

export GreedyOrderedAssignment

@with_kw struct GreedyOrderedAssignment{C,M,P} <: AbstractGreedyAssignment
    schedule::OperatingSchedule = OperatingSchedule()
    problem_spec::P             = ProblemSpec()
    cost_model::C               = SumOfMakeSpans()
    greedy_cost::M              = TaskGraphs.GreedyPathLengthCost()
    t0::Vector{Int}             = zeros(Int,nv(schedule))
    # backward_depth::Vector{Int} = compute_backward_depth(schedule)
    # ordering::Vector{Int}       = sortperm(backward_depth)
    ordering_graph::DiGraph     = get_graph(greedy_set_precedence_graph(schedule))
    ordering::Vector{Int}       = topological_sort_by_dfs(ordering_graph)
end

"""
    update_greedy_sets_enforce_order!(sched, cache, args...;kwargs...)

Requires that assignments be made in order--a task deeper in the schedule may 
not be assigned until all tasks less deep in the schedule have bee assigned.
TODO: Update this to better handle distinct project sub-trees. The key sorting 
criterion is not depth, but relative depth (i.e., don't make downstream 
assignments until the upstream assignments have been made).
"""
function update_greedy_sets_enforce_order!(sched, cache, 
        Ai=Set{Int}(), 
        Ao=Set{Int}(), 
        C=Set{Int}(), 
        ;
        backward_depth::Vector{Int}=compute_backward_depth(sched), 
        ordering::Vector{Int}=sortperm(backward_depth), 
        dmin::Int=nv(sched),
        start::Int=1
        )
    for v in Base.Iterators.rest(ordering,start)
        d = backward_depth[v]
        # Not allowed to make assignments deeper in the schedule until earlier
        # assignments have been made
        while (d > dmin)
            if (isempty(Ai) || isempty(Ao)) && length(C) < nv(sched)
                dmin += 1 # Prevent premature return of an empty assignment set
            else
                break
            end
        end
        if d <= dmin
            if issubset(inneighbors(sched,v),C)
                if indegree(sched,v) >= cache.n_required_predecessors[v]
                    push!(C,v)
                else
                    push!(Ai,v)
                end
            else
                dmin = d
            end
            if (outdegree(sched,v) < cache.n_eligible_successors[v]) && (v in C)
                push!(Ao,v)
            end
        else
            break
        end
    end
    @info "|Ai| = $(length(Ai)), |Ao| = $(length(Ao)), |C| = $(length(C)), nv(sched) = $(nv(sched)), dmin = $dmin"
    if (isempty(Ai) || isempty(Ao)) && length(C) < nv(sched)
        @warn "Assignment problem is infeasible"
    end
    return Ai, Ao, C
end

"""
    greedy_set_precedence_graph(sched)

Returns a graph whose edges represent precedence constraints on which nodes may
be added to the "eligible for assignment" sets in the course of an 
AbstractGreedyAssignment algorithm. The basic idea is to ensure that no 
`RobotGo` node is available for an incoming edge to be added until all 
assignments preceding that node's parent `OpenBuildStep` node have been made.
"""
function greedy_set_precedence_graph(sched)
    G = CustomNDiGraph{GraphUtils._node_type(sched),GraphUtils._id_type(sched)}()
    for n in get_nodes(sched)
        add_node!(G,n,node_id(n))
    end
    for edge in edges(sched)
        add_edge!(G,get_vtx_id(sched,edge.src),get_vtx_id(sched,edge.dst))
    end
    # G = DiGraph(nv(sched))
    # for edge in edges(sched)
    #     add_edge!(G,edge)
    # end
    close_step_map = backup_descendants(sched,n->matches_template(CloseBuildStep,n))
    open_step_map = backup_descendants(sched,n->matches_template(OpenBuildStep,n))
    close_step_groups = Dict{AbstractID,Vector{AbstractID}}() # node_id(CloseBuildStep) => [ancestor_ids...]
    for (k,id) in close_step_map
        if !(id === nothing)
            push!(get!(close_step_groups,id,valtype(close_step_groups)()),k)
        end
    end

    for n in filter(n->matches_template(OpenBuildStep,n), get_nodes(sched))
        close_node_id = close_step_map[node_id(n)]
        for id in close_step_groups[close_node_id]
            if !(open_step_map[id] == node_id(n))
                add_edge!(G,get_vtx(sched,node_id(n)),get_vtx(sched,id))
            end
        end
    end
    return G
end
greedy_set_precedence_ordering(sched) = topological_sort_by_dfs(greedy_set_precedence_graph(sched))

"""
    update_greedy_sets_ordered!(sched, cache, args...;kwargs...)

Ensured that that no `RobotGo` node is available for an incoming edge to be 
added until all assignments preceding that node's parent `OpenBuildStep` node 
have been made. This function is identical to `TaskGraphs.update_greedy_sets!` 
except that the graph being traversed is different from `model.schedule`.
"""
function update_greedy_sets_ordered!(sched, cache, 
        Ai=Set{Int}(), 
        Ao=Set{Int}(), 
        C=Set{Int}(), 
        ;
        ordering_graph::DiGraph=get_graph(greedy_set_precedence_graph(sched)),
        frontier::Set{Int}=get_all_root_nodes(ordering_graph),
        )
    while !isempty(frontier)
        v = pop!(frontier)
        if issubset(inneighbors(ordering_graph,v),C)
            if indegree(sched,v) >= cache.n_required_predecessors[v]
                push!(C,v)
                union!(frontier,outneighbors(ordering_graph,v))
            else
                push!(Ai,v)
            end
        end
        if (outdegree(sched,v) < cache.n_eligible_successors[v]) && (v in C)
            push!(Ao,v)
        end
    end
    @info "|Ai| = $(length(Ai)), |Ao| = $(length(Ao)), |C| = $(length(C)), nv(sched) = $(nv(sched))"
    return Ai, Ao, C
end

function TaskGraphs.update_greedy_sets!(model::GreedyOrderedAssignment,sched,cache,Ai=Set{Int}(),Ao=Set{Int}(),C=Set{Int}();
    kwargs...)
    # update_greedy_sets_enforce_order!(sched, cache, Ai, Ao, C;
    #     backward_depth=model.backward_depth,
    #     ordering=model.ordering,
    #     )
    update_greedy_sets_ordered!(sched, cache, Ai, Ao, C;
        ordering_graph=model.ordering_graph)

end
function TaskGraphs.formulate_milp(
        milp_model::GreedyOrderedAssignment,
        sched,
        problem_spec;
        cost_model=SumOfMakeSpans(),
        kwargs...
        )
    GreedyOrderedAssignment(
        schedule=sched,
        problem_spec=problem_spec,
        cost_model=cost_model
    )
end
function set_leaf_vtxs!(sched::OperatingSchedule,template=ProjectComplete)
    empty!(TaskGraphs.get_terminal_vtxs(sched))
    empty!(TaskGraphs.get_root_node_weights(sched))
    for v in vertices(sched)
        if is_terminal_node(sched,v) && matches_template(template,get_node(sched,v))
            push!(TaskGraphs.get_terminal_vtxs(sched),v)
            TaskGraphs.get_root_node_weights(sched)[v] = 1.0
        end
    end
    sched
end
function JuMP.optimize!(model::GreedyOrderedAssignment)
    TaskGraphs.greedy_assignment!(model)
    set_leaf_vtxs!(model.schedule,ProjectComplete)
end

# function greedy_assignment_with_ordered_build_steps!(model::GreedyOrderedAssignment,
#         sched=model.schedule,
#         scene_tree=model.problem_spec,
#         )
#     # Compute forward and backward depth 
#     cache = preprocess_project_schedule(sched,true)
#     backward_depth = model.backward_depth
#     ordering = model.ordering
#     # backward_depth = compute_backward_depth(sched)
#     # ordering = sortperm(backward_depth)
#     # Now require the order of assignments to respect the ordering of backward_depth
#     Ai,Ao,C = update_greedy_sets_enforce_order!(sched,cache;
#         backward_depth=backward_depth,
#         ordering=ordering)
#     D = TaskGraphs.construct_schedule_distance_matrix(sched,scene_tree)
#     while length(Ai) > 0
#         for (v,v2) in TaskGraphs.select_next_edges(model,D,Ao,Ai)
#             setdiff!(Ao,v)
#             setdiff!(Ai,v2)
#             add_edge!(sched,v,v2)
#             @info "$(string(node_id(get_node(sched,v)))), $(string(node_id(entity(get_node(sched,v))))) => $(string(node_id(get_node(sched,v2)))), $(string(node_id(entity(get_node(sched,v2)))))"
#         end
#         Ai,Ao,C = update_greedy_sets_enforce_order!(sched,cache,Ai,Ao,C,
#             backward_depth=backward_depth,ordering=ordering)
#     end
#     set_leaf_operation_vtxs!(sched)
#     propagate_valid_ids!(sched,scene_tree)
#     model
# end