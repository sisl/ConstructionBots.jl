duration_lower_bound(node::ConstructionPredicate) = 0.0

function duration_lower_bound(node::ConstructionPredicate, start, goal, max_speed)
    env_dt = DEFAULT_TIME_STEP
    d = norm(goal - start)
    δd = max_speed * env_dt
    num_steps = floor(d / δd)
    ϵ = capture_distance_tolerance()
    if abs(d - num_steps * δd) > ϵ  # eps(Float64)
        num_add_steps = ceil(ϵ / δd)
        num_steps += num_add_steps
    end
    return env_dt * num_steps
end

function duration_lower_bound(node::EntityGo)
    start = global_transform(start_config(node)).translation
    goal = global_transform(goal_config(node)).translation
    duration_lower_bound(node, start, goal, get_agent_max_speed(entity(node)))
end

function duration_lower_bound(node::Union{FormTransportUnit,DepositCargo})
    start = global_transform(cargo_start_config(node)).translation
    goal = global_transform(cargo_goal_config(node)).translation
    duration_lower_bound(node, start, goal, default_loading_speed())
end

function duration_lower_bound(node::LiftIntoPlace)
    # Need to account for order of movement: rotate, then translate
    env_dt = DEFAULT_TIME_STEP

    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()

    start_tform = global_transform(start_config(node))
    goal_tform = global_transform(goal_config(node))
    tf_error = relative_transform(start_tform, goal_tform)
    # Translation error
    dx = tf_error.translation[1:2]  
    trans_dt = 0.0
    if norm(dx) <= 1e-4
        trans_dt = env_dt
    else
        vel = normalize(dx) * min(v_max, norm(dx) / env_dt)
        trans_dt = duration_lower_bound(node, tf_error.translation[1:2], [0, 0], norm(vel))
    end
    # Rotation error (estimate for time)
    r = RotationVec(tf_error.linear)  # rotation vector
    θ = SVector(r.sx, r.sy, r.sz)  # convert r to svector
    if norm(θ) <= 1e-4
        rot_dt = 0.0
    else
        ω = normalize(θ) * min(ω_max, norm(θ) / env_dt)
        rot_dt = duration_lower_bound(node, θ, [0, 0, 0], norm(ω))
    end
    return trans_dt + rot_dt
end

is_tight(node::Union{TransportUnitGo,RobotGo}) = true

needs_path(node::BuildPhasePredicate) = false
needs_path(node::ObjectStart) = false
needs_path(node::AssemblyComplete) = false
needs_path(node::AssemblyStart) = false
needs_path(node::ProjectComplete) = false

function generate_path_spec(node::ConstructionPredicate)
    PathSpec(
        min_duration = duration_lower_bound(node),
        tight = is_tight(node),
        static = is_static(node),
        free = is_free(node),
        plan_path = needs_path(node),
    )
end

generate_path_spec(::SceneTree, node) = generate_path_spec(node)
generate_path_spec(::OperatingSchedule, scene_tree::SceneTree, node) =
    generate_path_spec(scene_tree, node)

function convert_to_operating_schedule(sched)
    tg_sched = OperatingSchedule()
    for node in get_nodes(sched)
        add_node!(
            tg_sched,
            ScheduleNode(node_id(node), node_val(node), generate_path_spec(node_val(node))),
        )
        if matches_template(ProjectComplete, node)
            push!(tg_sched.terminal_vtxs, get_vtx(tg_sched, node_id(node)))
        end
    end
    for e in edges(sched)
        add_edge!(tg_sched, get_vtx_id(sched, e.src), get_vtx_id(sched, e.dst))
    end
    tg_sched
end

function convert_from_operating_schedule(::Type{T}, sched::OperatingSchedule) where {T}
    G = T()
    for node in get_nodes(sched)
        add_node!(G, node.node, node_id(node))
    end
    for e in edges(sched)
        add_edge!(G, get_vtx_id(sched, e.src), get_vtx_id(sched, e.dst))
    end
    G
end

is_valid(node::SceneNode) = is_valid(node_id(node))

align_with_successor(node::ConstructionPredicate, succ::ConstructionPredicate) = node

align_with_successor(node::RobotGo, succ::T) where {T<:RobotGo} = RobotGo(
    first_valid(entity(node), entity(succ)),
    start_config(node),
    start_config(succ),
    node_id(node),
    nothing,
    nothing,
)

align_with_predecessor(
    node::RobotGo,
    pred::T,
) where {T<:Union{EntityConfigPredicate,EntityGo}} = RobotGo(
    first_valid(entity(node), entity(pred)),
    goal_config(pred),
    goal_config(node),
    node_id(node),
    nothing,
    nothing,
)

ALIGNMENT_CHECK_TOLERANCE = 1e-3

function get_matching_child_id(node::FormTransportUnit, pred::RobotGo)
    t_rel = relative_transform(
        global_transform(goal_config(node)),
        global_transform(goal_config(pred)),
    )
    transport_unit = entity(node)
    for id in collect(keys(robot_team(transport_unit)))
        tform = child_transform(transport_unit, id)
        if norm(tform.translation - t_rel.translation) <= 1e-3
            return id
        end
    end
    return nothing
end

function get_matching_child_id(node::DepositCargo, succ::RobotGo)
    t_rel = relative_transform(
        global_transform(goal_config(node)),
        global_transform(start_config(succ)),
    )
    transport_unit = entity(node)
    for id in collect(keys(robot_team(transport_unit)))
        tform = child_transform(transport_unit, id)
        if norm(tform.translation - t_rel.translation) <= 1e-3
            return id
        end
    end
    return nothing
end

function align_with_predecessor(node::FormTransportUnit, pred::RobotGo)
    matching_id = get_matching_child_id(node, pred)
    if !(matching_id === nothing)
        transport_unit = entity(node)
        swap_robot_id!(transport_unit, matching_id, node_id(entity(pred)))
    end
    node
end

function align_with_predecessor(sched::OperatingSchedule, node::RobotGo, pred::DepositCargo)
    matching_id = get_matching_child_id(pred, node)
    if !(matching_id === nothing)
        if valid_id(matching_id)
            # Have to pull the RobotNode from sched, because it's not accessible through DepositCargo
            robot_start = get_node(sched, RobotStart(RobotNode(matching_id, entity(node))))
            return RobotGo(
                RobotNode(matching_id, entity(robot_start)),
                start_config(node),
                goal_config(node),
                node_id(node),
                nothing,
                nothing,
            )
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
function build_and_link_dummy_robot!(node, id, tform)
    robot_go = RobotGo(RobotNode(id, GeomNode(nothing)))
    set_parent!(goal_config(robot_go), goal_config(node))
    set_parent!(start_config(robot_go), goal_config(robot_go))
    set_local_transform!(goal_config(robot_go), tform)
    robot_go
end

function add_dummy_robot_go_nodes!(sched)
    for node in get_nodes(sched)
        if matches_template(FormTransportUnit, node)
            transport_unit = entity(node)
            for (id, tform) in robot_team(transport_unit)
                robot_go = build_and_link_dummy_robot!(node, id, tform)
                add_node!(sched, robot_go)
                @assert add_edge!(sched, robot_go, node)
            end
        elseif matches_template(DepositCargo, node)
            transport_unit = entity(node)
            for (id, tform) in robot_team(transport_unit)
                robot_go = build_and_link_dummy_robot!(node, id, tform)
                # add_node!(sched, robot_go)
                add_node!(sched, robot_go)
                @assert add_edge!(sched, node, robot_go)
            end
        end
    end
end

function compute_backward_depth(sched)
    forward_depth = zeros(Int, nv(sched))
    for v in topological_sort_by_dfs(sched)
        forward_depth[v] =
            maximum([0, map(vp -> forward_depth[vp] + 1, inneighbors(sched, v))...])
    end
    backward_depth = deepcopy(forward_depth)
    for v in reverse(topological_sort_by_dfs(sched))
        backward_depth[v] = maximum([
            backward_depth[v],
            map(vp -> backward_depth[vp] - 1, outneighbors(sched, v))...,
        ])
    end
    return backward_depth
end

export GreedyOrderedAssignment

@with_kw struct GreedyOrderedAssignment{C,M,P} <: AbstractGreedyAssignment
    schedule::OperatingSchedule = OperatingSchedule()
    problem_spec::P = ProblemSpec()
    cost_model::C = SumOfMakeSpans()
    greedy_cost::M = GreedyPathLengthCost()
end

function formulate_milp(
    milp_model::GreedyOrderedAssignment,
    sched,
    problem_spec;
    cost_model = SumOfMakeSpans(),
    kwargs...,
)
    GreedyOrderedAssignment(
        schedule = sched,
        problem_spec = problem_spec,
        cost_model = cost_model,
        greedy_cost = milp_model.greedy_cost,
    )
end

function set_leaf_vtxs!(sched::OperatingSchedule, template = ProjectComplete)
    empty!(get_terminal_vtxs(sched))
    empty!(get_root_node_weights(sched))
    for v in Graphs.vertices(sched)
        if is_terminal_node(sched, v) && matches_template(template, get_node(sched, v))
            push!(get_terminal_vtxs(sched), v)
            get_root_node_weights(sched)[v] = 1.0
        end
    end
    sched
end

function JuMP.optimize!(model::GreedyOrderedAssignment)
    assign_collaborative_tasks!(model)
    set_leaf_vtxs!(model.schedule, ProjectComplete)
end

function construct_build_step_task_set(sched, scene_tree, step_id)
    tasks = Set{AbstractID}()
    n = get_node(sched, step_id)
    for (part_id, tform) in assembly_components(n.node)
        tu = get_node(scene_tree, TransportUnitNode(get_node(scene_tree, part_id)))
        form_tu_node = get_node(sched, FormTransportUnit(tu))
        push!(tasks, node_id(form_tu_node))
    end
    tasks
end

function build_step_dependency_graph(sched, scene_tree)
    dependency_graph = DiGraph(nv(sched))  # track robot -> build step edges
    # Add edges between build steps and assemblies
    for n in get_nodes(sched)
        v = get_vtx(sched, n)
        if matches_template(CloseBuildStep, n)
            close_step_vtx = v
            open_step_vtx = get_vtx(sched, OpenBuildStep(n.node))
            add_edge!(dependency_graph, open_step_vtx, close_step_vtx)  # open -> close
            next_vtx = first(outneighbors(sched, n))
            add_edge!(dependency_graph, close_step_vtx, next_vtx)  # close -> next vtx (OpenBuildStep or AssemblyComplete)
            for (part_id, tform) in assembly_components(n.node)
                part = get_node(scene_tree, part_id)
                if matches_template(AssemblyID, part_id)
                    start_vtx = get_vtx(sched, AssemblyComplete(part))
                else
                    start_vtx = get_vtx(sched, ObjectStart(part))
                end
                transport_vtx = get_vtx(sched, FormTransportUnit(TransportUnitNode(part)))
                add_edge!(dependency_graph, start_vtx, transport_vtx)
                add_edge!(dependency_graph, open_step_vtx, transport_vtx)
                add_edge!(dependency_graph, transport_vtx, close_step_vtx)
            end
        end
    end
    dependency_graph
end

"""
    assign_collaborative_tasks!(model)

Assign collaborative tasks in a greedy manner.
"""
function assign_collaborative_tasks!(model)
    sched = model.schedule
    scene_tree = model.problem_spec
    assembly_starts = Dict(
        node_id(n) => n for n in get_nodes(sched) if matches_template(AssemblyStart, n)
    )
    assemblies_completed = Set{AbstractID}()
    active_build_steps = Dict()
    for (k, n) in assembly_starts
        open_build_step = get_first_build_step(sched, n)
        step_id = node_id(open_build_step)
        active_build_steps[step_id] =
            construct_build_step_task_set(sched, scene_tree, step_id)
    end
    # Fill robots with go nodes
    robots = Set{Int}()
    for n in get_nodes(sched)
        if matches_template(RobotStart, n)
            go_vtx = first(outneighbors(sched, n))
            @assert isempty(outneighbors(sched, go_vtx))
            push!(robots, go_vtx)
        end
    end
    robots, active_build_steps
    # Assign tasks
    NUM_BUILD_STEPS =
        length([true for n in get_nodes(sched) if matches_template(OpenBuildStep, n)])
    BUILD_STEPS_CLOSED = 0
    @info "Beginning task assignment"
    # Initialize dependency graph to track and prevent cycles
    dependency_graph = build_step_dependency_graph(sched, scene_tree)
    distance_dict = Dict{Tuple{Int,Int},Float64}()
    cost_func =
        (v, v2) -> begin
            if !haskey(distance_dict, (v, v2))
                new_node = align_with_successor(
                    get_node(sched, v).node,
                    get_node(sched, v2).node,
                )
                distance_dict[(v, v2)] =
                    generate_path_spec(sched, scene_tree, new_node).min_duration
            end
            return get_tF(sched, v) + distance_dict[(v, v2)]
        end
    while !isempty(active_build_steps)
        # Get best possible assignment of robots to a team task
        best_cost = Inf
        build_step_id = nothing
        task_id = nothing
        best_assignments = nothing
        for (step_id, tasks) in active_build_steps
            step_vtx = get_vtx(sched, step_id)
            for task in tasks
                task_node = get_node(sched, task)
                cargo = get_node(scene_tree, cargo_id(entity(task_node)))
                if matches_template(AssemblyNode, cargo) &&
                   !(node_id(cargo) in assemblies_completed)
                    continue
                end
                team_slots = Set(
                    v for v in inneighbors(sched, task) if
                    matches_template(RobotGo, get_node(sched, v))
                )
                assignments = []
                filt_func = (v, v2) -> !has_path(dependency_graph, get_vtx(sched, task), v)
                cost = 0.0
                while !isempty(team_slots)
                    robot, slot, c = get_best_pair(robots, team_slots, cost_func, filt_func)
                    cost = max(c, cost)
                    if cost >= best_cost
                        break
                    end
                    push!(assignments, robot => slot)
                    @assert slot in team_slots
                    @assert robot in robots
                    setdiff!(team_slots, slot)  # remove slot from team_slots
                    setdiff!(robots, robot)  # remove robots from robots
                end
                # @info "assignment for $(summary(task)): $assignments"
                # Replace robot in robot set
                for (robot, slot) in assignments
                    push!(robots, robot)
                end
                # Update best assignment
                if cost < best_cost
                    build_step_id = step_id
                    task_id = task
                    best_assignments = assignments
                    best_cost = cost
                end
            end
        end
        # Update schedule
        @assert !(best_assignments === nothing) "no assignment found!"
        team_task_vtx = get_vtx(sched, task_id)
        open_step_vtx = get_vtx(sched, build_step_id)
        close_step_vtx = get_vtx(sched, CloseBuildStep(get_node(sched, build_step_id).node))
        for (robot, task) in best_assignments
            add_edge!(sched, robot, task)
            # Add RobotNode -> TeamTask and OpenBuildStep -> TeamTask edges to dependency graph.
            add_edge!(dependency_graph, robot, team_task_vtx)
            add_edge!(dependency_graph, open_step_vtx, team_task_vtx)
            add_edge!(dependency_graph, team_task_vtx, close_step_vtx)
            setdiff!(robots, robot)  # remove robots
        end
        # Add new robots
        deposit_node = get_node(sched, DepositCargo(entity(get_node(sched, task_id))))
        for v in outneighbors(sched, deposit_node)
            if matches_template(RobotGo, get_node(sched, v))
                robot = v
                push!(robots, robot)
                # Add TeamTask -> RobotNode edge to dependency graph
                add_edge!(dependency_graph, team_task_vtx, robot)
            end
        end
        # Update schedule times
        update_schedule_times!(sched)
        # Update active build step list
        delete!(active_build_steps[build_step_id], task_id)
        if isempty(active_build_steps[build_step_id])
            BUILD_STEPS_CLOSED += 1
            @info "Assignment: closing build step $(summary(build_step_id)). $(BUILD_STEPS_CLOSED)/$(NUM_BUILD_STEPS) complete. "
            delete!(active_build_steps, build_step_id)
            close_build_step =
                get_node(sched, CloseBuildStep(get_node(sched, build_step_id).node))
            next_node = get_node(sched, first(outneighbors(sched, close_build_step)))
            if matches_template(OpenBuildStep, next_node)
                step_id = node_id(next_node)
                active_build_steps[step_id] =
                    construct_build_step_task_set(sched, scene_tree, step_id)
                # @info "new build step $(summary(step_id)) with tasks $(active_build_steps[step_id])"
            else
                # Assembly complete
                @assert matches_template(AssemblyComplete, next_node)
                push!(assemblies_completed, node_id(entity(next_node)))
                @info "Closing Assembly $(summary(node_id(entity(next_node))))"
            end
        end
    end
    @info "Assignment Complete!"
    model
end
