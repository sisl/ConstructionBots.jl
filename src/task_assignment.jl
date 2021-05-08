TaskGraphs.duration_lower_bound(node::ConstructionPredicate) = 0.0
function TaskGraphs.duration_lower_bound(node::ConstructionPredicate,start,goal,max_speed)
    d = norm(goal-start)
    dt = d/max_speed
end
function TaskGraphs.duration_lower_bound(node::EntityGo)
    start = global_transform(start_config(node)).translation
    goal = global_transform(goal_config(node)).translation
    TaskGraphs.duration_lower_bound(node,start,goal,get_rvo_max_speed(entity(node)))
end
function TaskGraphs.duration_lower_bound(node::Union{FormTransportUnit,DepositCargo})
    start = global_transform(cargo_start_config(node)).translation
    goal = global_transform(cargo_goal_config(node)).translation
    TaskGraphs.duration_lower_bound(node,start,goal,default_loading_speed())
    # d = norm(goal-start)
    # max_speed = default_loading_speed()
    # dt = d/max_speed
end
function TaskGraphs.duration_lower_bound(node::LiftIntoPlace)
    start = global_transform(start_config(node)).translation
    goal = global_transform(goal_config(node)).translation
    TaskGraphs.duration_lower_bound(node,start,goal,default_loading_speed())
    # d = norm(goal-start)
    # max_speed = default_loading_speed()
    # dt = d/max_speed
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
    t0::Vector{Float64}         = get_tF(schedule)
    # ordering_graph::DiGraph     = get_graph(greedy_set_precedence_graph(schedule))
    ordering_graph::DiGraph     = get_graph(construct_build_step_graph(schedule))
    ordering::Vector{Int}       = topological_sort_by_dfs(ordering_graph)
    frontier::Set{Int}          = get_all_root_nodes(ordering_graph)
end

"""
    construct_build_step_graph(sched)

Returns a graph with the same nodes as sched, but with edges modified to enforce
precedence (for assignment) between the tasks associated with each build phase. 
"""
function construct_build_step_graph(sched)
    build_step_graph = CustomNDiGraph{GraphUtils._node_type(sched),GraphUtils._id_type(sched)}()
    # build step dependency graph
    for n in get_nodes(sched)
        add_node!(build_step_graph,n,node_id(n))
    end
    for n in get_nodes(build_step_graph)
        val = n.node
        if matches_template(CloseBuildStep,n)
            ob = get_node(build_step_graph,OpenBuildStep(val))
            add_edge!(build_step_graph,ob,n)
        end
    end
    # subassembly dependencies
    for n in get_nodes(build_step_graph)
        val = n.node
        if matches_template(CloseBuildStep,n)
            blanket = GraphUtils.backward_cover(sched,get_vtx(sched,n),np->matches_template(CloseBuildStep,np))
            for np in blanket
                if node_id(np) != node_id(n)
                    add_edge!(build_step_graph,np,OpenBuildStep(val))
                end
            end
        end
    end
    # add tasks
    for n in get_nodes(build_step_graph)
        if matches_template(CloseBuildStep,n)
            cb = n
            ob = get_node(sched,OpenBuildStep(n.node))
            for component in keys(assembly_components(n.node))
                if matches_template(ObjectID,component)
                    transport_unit = TransportUnitNode(ObjectNode(component,GeomNode(nothing)))
                else
                    transport_unit = TransportUnitNode(AssemblyNode(component,GeomNode(nothing)))
                end
                form_transport_unit = get_node(sched,FormTransportUnit(transport_unit))
                for np in node_iterator(sched,inneighbors(sched,form_transport_unit))
                    if matches_template(RobotGo,np)
                        add_edge!(build_step_graph,ob,np)
                        add_edge!(build_step_graph,np,cb)
                    end
                end
                deposit_cargo = get_node(sched,DepositCargo(transport_unit))
                for np in node_iterator(sched,outneighbors(sched,deposit_cargo))
                    if matches_template(RobotGo,np)
                        add_edge!(build_step_graph,cb,np)
                    end
                end
            end
        end
    end
    build_step_graph
end

# """
#     update_greedy_sets_enforce_order!(sched, cache, args...;kwargs...)

# Requires that assignments be made in order--a task deeper in the schedule may 
# not be assigned until all tasks less deep in the schedule have been assigned.
# TODO: Update this to better handle distinct project sub-trees. The key sorting 
# criterion is not depth, but relative depth (i.e., don't make downstream 
# assignments until the upstream assignments have been made).
# """
# function update_greedy_sets_enforce_order!(sched, cache, 
#         Ai=Set{Int}(), 
#         Ao=Set{Int}(), 
#         C=Set{Int}(), 
#         ;
#         backward_depth::Vector{Int}=compute_backward_depth(sched), 
#         ordering::Vector{Int}=sortperm(backward_depth), 
#         dmin::Int=nv(sched),
#         start::Int=1
#         )
#     for v in Base.Iterators.rest(ordering,start)
#         d = backward_depth[v]
#         # Not allowed to make assignments deeper in the schedule until earlier
#         # assignments have been made
#         while (d > dmin)
#             if (isempty(Ai) || isempty(Ao)) && length(C) < nv(sched)
#                 dmin += 1 # Prevent premature return of an empty assignment set
#             else
#                 break
#             end
#         end
#         if d <= dmin
#             if issubset(inneighbors(sched,v),C)
#                 if indegree(sched,v) >= cache.n_required_predecessors[v]
#                     push!(C,v)
#                 else
#                     push!(Ai,v)
#                 end
#             else
#                 dmin = d
#             end
#             if (outdegree(sched,v) < cache.n_eligible_successors[v]) && (v in C)
#                 push!(Ao,v)
#             end
#         else
#             break
#         end
#     end
#     @info "|Ai| = $(length(Ai)), |Ao| = $(length(Ao)), |C| = $(length(C)), nv(sched) = $(nv(sched)), dmin = $dmin"
#     if (isempty(Ai) || isempty(Ao)) && length(C) < nv(sched)
#         @warn "Assignment problem is infeasible"
#     end
#     TaskGraphs.process_schedule!(sched)
#     return Ai, Ao, C
# end

# """
#     greedy_set_precedence_graph(sched)

# Returns a graph whose edges represent precedence constraints on which nodes may
# be added to the "eligible for assignment" sets in the course of an 
# AbstractGreedyAssignment algorithm. The basic idea is to ensure that no 
# `RobotGo` node is available for an incoming edge to be added until all 
# assignments preceding that node's parent `OpenBuildStep` node have been made.
# """
# function greedy_set_precedence_graph(sched)
#     G = CustomNDiGraph{GraphUtils._node_type(sched),GraphUtils._id_type(sched)}()
#     # copy graph topology
#     for n in get_nodes(sched)
#         add_node!(G,n,node_id(n))
#     end
#     for edge in edges(sched)
#         add_edge!(G,get_vtx_id(sched,edge.src),get_vtx_id(sched,edge.dst))
#     end
#     close_step_map = backup_descendants(sched,n->matches_template(CloseBuildStep,n))
#     open_step_map = backup_descendants(sched,n->matches_template(OpenBuildStep,n))
#     close_step_groups = Dict{AbstractID,Vector{AbstractID}}() # node_id(CloseBuildStep) => [ancestor_ids...]
#     for (k,id) in close_step_map
#         if !(id === nothing)
#             push!(get!(close_step_groups,id,valtype(close_step_groups)()),k)
#         end
#     end
#     for n in filter(n->matches_template(OpenBuildStep,n), get_nodes(sched))
#         close_node_id = close_step_map[node_id(n)]
#         for id in close_step_groups[close_node_id]
#             if !(open_step_map[id] == node_id(n))
#                 add_edge!(G,get_vtx(sched,node_id(n)),get_vtx(sched,id))
#             end
#         end
#     end
#     # for n in filter(n->matches_template(CloseBuildStep,n), get_nodes(sched))
#     #     blanket = GraphUtils.backward_cover(sched,node_id(n),np->matches_template(CloseBuildStep,np))
#     #     for np in blanket
#     #         add_edge!(G,np,OpenBuildStep(n.node))
#     #     end
#     # end
#     # for n in filter(n->matches_template(OpenBuildStep,n), get_nodes(sched))
#     #     for component in keys(assembly_components(n.node))
#     #         if matches_template(ObjectID,component)
#     #             cargo = ObjectNode(component,GeomNode(nothing))
#     #         else
#     #             @assert (matches_template(AssemblyID,component))
#     #             cargo = AssemblyNode(component,GeomNode(nothing))
#     #         end
#     #         form_transport_unit = get_node(sched,FormTransportUnit(TransportUnitNode(cargo)))
#     #         for np in node_iterator(sched,inneighbors(sched,form_transport_unit))
#     #             if matches_template(RobotGo,np)
#     #                 add_edge!(G,n,np)
#     #             end
#     #         end
#     #     end
#     # end
#     return G
# end
# greedy_set_precedence_ordering(sched) = topological_sort_by_dfs(greedy_set_precedence_graph(sched))
function construct_build_step_task_set(sched,scene_tree,step_id)
    tasks = Set{AbstractID}()
    n = get_node(sched,step_id)
    for (part_id,tform) in assembly_components(n.node)
        tu = get_node(scene_tree,TransportUnitNode(get_node(scene_tree,part_id)))
        form_tu_node = get_node(sched,FormTransportUnit(tu))
        push!(tasks,node_id(form_tu_node))
    end
    tasks
end

function assign_collaborative_tasks!(model)
    sched       = model.schedule
    scene_tree  = model.problem_spec
    D = TaskGraphs.construct_schedule_distance_matrix(sched,scene_tree)
    assembly_starts = Dict(node_id(n)=>n for n in get_nodes(sched) if matches_template(AssemblyStart,n))
    active_build_steps = Dict()
    for (k,n) in assembly_starts
        open_build_step = get_first_build_step(sched,n) 
        step_id = node_id(open_build_step)
        # active_build_steps[node_id(open_build_step)] = Set{AbstractID}()
        active_build_steps[step_id] = construct_build_step_task_set(sched,scene_tree,step_id)
    end
    # fill robots with go nodes
    robots = Set{Int}()
    for n in get_nodes(sched)
        if matches_template(RobotStart,n)
            go_vtx = first(outneighbors(sched,n))
            @assert isempty(outneighbors(sched,go_vtx))
            push!(robots,go_vtx)
        end
    end
    robots, active_build_steps
    # assign tasks

    @info "Beginning task assignment"
    while !isempty(active_build_steps)
        # get best possible assignment of robots to a team task
        best_cost       = Inf
        build_step_id   = nothing
        task_id         = nothing
        best_assignments = nothing
        for (step_id,tasks) in active_build_steps
            for task in tasks
                team_slots = Set(v for v in inneighbors(sched,task) if matches_template(RobotGo,get_node(sched,v)))
                assignments = []
                cost = 0.0
                while !isempty(team_slots)
                    robot, slot, c = TaskGraphs.get_best_pair(robots, team_slots,
                        (v,v2)->TaskGraphs.get_edge_cost(model,D,v,v2))
                    cost = max(c,cost)
                    if cost > best_cost
                        break
                    end
                    push!(assignments,robot=>slot)
                    setdiff!(team_slots,slot) # remove slot from team_slots
                    setdiff!(robots,robot) # remove robots from robots
                end
                # @info "assignment for $(summary(task)): $assignments"
                # replace robot in robot set
                for (robot,slot) in assignments
                    push!(robots,robot)
                end
                # update best assignment
                if cost < best_cost
                    build_step_id = step_id
                    task_id = task
                    best_assignments = assignments
                    best_cost = cost
                    # @info "updating best assignment to $(summary(task)): $assignments"
                end
            end
        end
        # update schedule
        @assert !(best_assignments === nothing) "no assignment found!"
        # @info "best assignment selected $(summary(task_id)): $best_assignments"
        for (robot,task) in best_assignments
            add_edge!(sched,robot,task)
            setdiff!(robots,robot) # remove robots
        end
        # add new robots
        deposit_node = get_node(sched,DepositCargo(entity(get_node(sched,task_id))))
        for v in outneighbors(sched,deposit_node) 
            if matches_template(RobotGo,get_node(sched,v))
                push!(robots,v)
            end
        end
        # update schedule times
        TaskGraphs.update_schedule_times!(sched)
        # update active build step list
        delete!(active_build_steps[build_step_id], task_id)
        if isempty(active_build_steps[build_step_id])
            @info "Assignment: closing build step $(summary(build_step_id))"
            delete!(active_build_steps, build_step_id)
            close_build_step = get_node(sched,CloseBuildStep(get_node(sched,build_step_id).node))
            next_build_step = get_node(sched,first(outneighbors(sched,close_build_step)))
            if matches_template(OpenBuildStep,next_build_step)
                step_id = node_id(next_build_step)
                active_build_steps[step_id] = construct_build_step_task_set(
                    sched,
                    scene_tree,
                    step_id)
                # @info "new build step $(summary(step_id)) with tasks $(active_build_steps[step_id])"
            end
        end
    end
    @info "Assignment Complete!"
    model
end

function get_best_assignment!(model,sched,scene_tree,task,robot_ids,D)
    goal_vtxs = inneighbors(sched,task)
    robot_vtxs = [get_vtx(sched,id) for id in robot_ids]
    c = Inf
    robot = nothing
    goal = nothing
    for v2 in goal_vtxs
        for v in robot_vtxs
            cost = get_edge_cost(model,D,v,v2)
            if cost < c
                c = cost
                robot = v
                goal = v2
            end
        end
    end
    return robot, goal
end

"""
    update_greedy_sets_ordered!(model,sched, cache, args...;kwargs...)

Ensured that that no `RobotGo` node is available for an incoming edge to be 
added until all assignments preceding that node's parent `OpenBuildStep` node 
have been made. This function is identical to `TaskGraphs.update_greedy_sets!` 
except that the graph being traversed is different from `model.schedule`.
"""
function update_greedy_sets_ordered!(model,sched, cache, 
        Ai=Set{Int}(), 
        Ao=Set{Int}(), 
        C=Set{Int}(), 
        ;
        ordering_graph::DiGraph=get_graph(greedy_set_precedence_graph(sched)),
        frontier::Set{Int}=model.frontier,
        # frontier::Set{Int}=get_all_root_nodes(ordering_graph),
        )
    explored = Set{Int}()
    ordered_frontier = Vector{Int}(collect(frontier))
    t = time()
    # TODO ensure that we don't get stuck with a bunch of half-full team transport assignments
    while !isempty(ordered_frontier)
        v = popfirst!(ordered_frontier)
        if issubset(inneighbors(ordering_graph,v),C)
            if indegree(sched,v) >= cache.n_required_predecessors[v]
                push!(C,v)
                for vp in outneighbors(ordering_graph,v)
                    push!(ordered_frontier,vp)
                end
            else
                push!(Ai,v)
            end
        end
        if (outdegree(sched,v) < cache.n_eligible_successors[v]) && (v in C)
            push!(Ao,v)
        end
        push!(explored,v)
        if issubset(outneighbors(ordering_graph,v),C)
            setdiff!(model.frontier,v)
        else
            push!(model.frontier,v)
        end
    end
    @info "update took $(time()-t) seconds"
    @info "|Ai| = $(length(Ai)), |Ao| = $(length(Ao)), |C| = $(length(C)), nv(sched) = $(nv(sched))"
    return Ai, Ao, C
end

"""
    select_next_edges(model::GreedyOrderedAssignment,D,Ao,Ai)

Fill collaborative tasks all at once.
"""
function TaskGraphs.select_next_edges(model::GreedyOrderedAssignment,D,Ao,Ai)
    sched = model.schedule
    best_edges = TaskGraphs.select_next_edges(
        TaskGraphs.edge_selection_model(model),model,D,Ao,Ai)
    edge_list = deepcopy(best_edges)
    while isempty(best_edges)
        # check if collaborative task. If so, fill up immediately
        (a,b) = pop!(best_edges)
        setdiff!(Ao,a)
        setdiff!(Ai,b)
        dst = get_node(sched,b)
        for n in node_iterator(sched,outneighbors(sched,dst))
            if matches_template(FormTransportUnit,n)
                team_size = length(robot_team(entity(n)))
                if team_size > 1
                    # is collaborative
                    if team_size > length(Ao) 
                        # not enough robots available to make this assignment right now
                        # put `a` back and get a new assignment
                        push!(Ao,a)
                        best_edges = TaskGraphs.select_next_edges(
                            TaskGraphs.edge_selection_model(model),model,D,Ao,Ai)
                    end
                    # fill up remainder of team member assignments immediately
                    @info "filling assignments for $(summary(node_id(n))) with team size $(length(robot_team(entity(n)))). |Ao| = $(length(Ao))"
                    edge_list = fill_assignments_greedy!(
                        inneighbors(sched,n), model, sched, D, Ai, Ao)
                    @info "completed collaborative transport assignments for $(summary(node_id(n))): $(edge_list)"
                    best_edges = vcat(best_edges,edge_list)
                end
            end
        end
    end
    best_edges
end

"""
    fill_assignments_greedy!(vtxs,model,sched,D,Ai,Ao)

Greedily fill all assignments for the vertices in `vtxs`
"""
function fill_assignments_greedy!(vtxs,model,sched,D,Ai,Ao)
    edge_list = []
    for v2 in vtxs
        if v2 in Ai
            c = Inf
            a = -1
            for v in sort(collect(Ao))
                cost = TaskGraphs.get_edge_cost(model,D,v,v2)
                if cost < c
                    c = cost
                    a = v
                end
            end
            if has_vertex(sched,a)
                push!(edge_list,(a,v2))
                setdiff!(Ai,v2)
                setdiff!(Ao,a)
            else
                @warn "Failed to fill up transport unit" Ai Ao v2
                throw(ErrorException("GreedyAssignment is stuck"))
            end
        end
    end
    edge_list
end

function TaskGraphs.update_greedy_sets!(model::GreedyOrderedAssignment,sched,cache,Ai=Set{Int}(),Ao=Set{Int}(),C=Set{Int}();
    # frontier=Set{Int}(),
    kwargs...,
    )
    # @show frontier
    update_greedy_sets_ordered!(model,sched, cache, Ai, Ao, C;
        ordering_graph=model.ordering_graph,
        # frontier=union(frontier,Ao)
        # kwargs...
        )

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
        cost_model=cost_model,
        greedy_cost = milp_model.greedy_cost,
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

# global GREEDY_UPDATE_COUNTER = 10
# function TaskGraphs.update_greedy_cost_model!(model::GreedyOrderedAssignment,args...) 
#     TaskGraphs.update_schedule_times!(model.schedule)
#     # global GREEDY_UPDATE_COUNTER
#     # if GREEDY_UPDATE_COUNTER == 0
#     #     GREEDY_UPDATE_COUNTER = 10
#     #     TaskGraphs.update_schedule_times!(model.schedule)
#     # else
#     #     GREEDY_UPDATE_COUNTER -= 1
#     # end
#     # return nothing
#     # update_greedy_cost_model!(model.greedy_cost,model,args...) 
# end