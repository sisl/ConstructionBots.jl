# module Replanning
#
# using Parameters
# using LightGraphs
# using GraphUtils
# using CRCBS
# using ..TaskGraphs

using Printf: @sprintf

export
    ProjectRequest

"""
    ProjectRequest

Encodes a "request" that arrives in the factory command center.
    `t_request` encodes the time at which the request reaches the command center
    `t_arrival` is the time at which the project materials will be available

TODO more flexibility with t_arrival by allowing different materials to arrive
at different times
"""
@with_kw struct ProjectRequest
    schedule::OperatingSchedule = OperatingSchedule()
    t_request::Int              = -1
    t_arrival::Int              = -1
end
function project_request(args...)
    req = ProjectRequest(args...)
    set_t0!(req,req.t_request)
    process_schedule!(req.schedule)
    return req
end

export
    RepeatedAbstractPC_TAPF,
    RepeatedPC_TAPF,
    RepeatedC_PC_TAPF

abstract type RepeatedAbstractPC_TAPF end
CRCBS.get_env(prob::RepeatedAbstractPC_TAPF) = prob.env

"""
    RepeatedPC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF

Encodes a Repeated PC_TAPF problem.
Usage:
    `prob = RepeatedPC_TAPF(env::SearchEnv,requests::Vector{ProjectRequest})`
"""
struct RepeatedPC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF
    env::S
    requests::Vector{ProjectRequest}
end
RepeatedPC_TAPF(env::SearchEnv) = RepeatedPC_TAPF(env,Vector{ProjectRequest}())
struct RepeatedC_PC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF
    env::S
    requests::Vector{ProjectRequest}
end
RepeatedC_PC_TAPF(env::SearchEnv) = RepeatedC_PC_TAPF(env,Vector{ProjectRequest}())

for T in [:RepeatedPC_TAPF,:RepeatedC_PC_TAPF]
    @eval $T(prob::$T,env::SearchEnv) = $T(env,prob.requests)
end

construct_pctapf_problem(prob::RepeatedPC_TAPF,env) = PC_TAPF(env)
construct_pctapf_problem(prob::RepeatedC_PC_TAPF,env) = C_PC_TAPF(env)


"""
    trim_route_plan(search_env, route_plan, T)

Construct a trimmed route_plan that stops at a certain time step
"""
function trim_route_plan(search_env, route_plan, T)
    trim_solution!(build_env(search_env),route_plan,T)
end

export
    get_active_and_fixed_vtxs,
    get_active_vtxs,
    get_fixed_vtxs,
    split_active_vtxs!,
    fix_precutoff_nodes!,
    break_assignments!,
    prune_schedule,
    # prune_project_schedule,
    splice_schedules!

"""
    get_active_and_fixed_vtxs(sched::OperatingSchedule,t)

"active" vertices "straddle" the query time t
"fixed" vertices finish before the query time t
"""
function get_active_and_fixed_vtxs(sched::OperatingSchedule,t)
    active_vtxs = Set{Int}()
    fixed_vtxs = Set{Int}()
    for v in vertices(get_graph(sched))
        t0 = get_t0(sched,v)
        tF = get_tF(sched,v)
        # t0 = Int(round(get_t0(sched,v)))
        # tF = Int(round(get_tF(sched,v)))
        if tF + minimum(get_local_slack(sched,v)) <= t
            push!(fixed_vtxs, v)
        elseif t0 <= t < tF + minimum(get_local_slack(sched,v))
            push!(active_vtxs, v)
        end
    end
    @assert all(map(v->get_tF(sched,v) + minimum(get_local_slack(sched,v)), collect(active_vtxs)) .>= t)
    active_vtxs, fixed_vtxs
end
get_active_vtxs(sched,t) = get_active_and_fixed_vtxs(sched,t)[1]
get_fixed_vtxs(sched,t) = get_active_and_fixed_vtxs(sched,t)[2]

function split_action_node!(sched::OperatingSchedule,problem_spec::ProblemSpec,node::ScheduleNode,x,t)
    @assert isa(node.id,ActionID)
    pred1,pred2 = split_node(node.node,x)
    node1 = replace_in_schedule!(sched,pred1,node.id)
    set_t0!(node1,get_t0(node))
    set_tF!(node1,t)
    node2 = add_node!(sched,make_node(sched,problem_spec,pred2))
    set_t0!(node2,t)
    set_tF!(node2,t+get_min_duration(node2.spec))
    # @show node_id(node1), string(node1.node), node1
    # @show node_id(node2), string(node2.node), node2
    # set_tF!(sched,id2,get_tF(node))
    for v in outneighbors(sched,get_vtx(sched,node1.id))
        rem_edge!(sched,node1,v)
        add_edge!(sched,node2,v)
    end
    add_edge!(sched,node1,node2)
    node1,node2
end

"""
    split_active_vtxs!(sched::OperatingSchedule,
        problem_spec::ProblemSpec,t;

Split all GO nodes that "straddle" the cutoff time.
"""
function split_active_vtxs!(sched::OperatingSchedule,problem_spec::ProblemSpec,t;
    robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
    )
    G = get_graph(sched)
    # identify nodes "cut" by timestep t
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(sched,t)
    # split active nodes
    for v in active_vtxs
        node = get_node(sched,v)
        if matches_template(BOT_GO,node) # split TODO why aren't we splitting CARRY, COLLECT, and DEPOSIT as well?
            x = robot_positions[get_robot_id(node)].x
            node1,node2 = split_action_node!(sched,problem_spec,node,x,t)
            set_path_spec!(node1,PathSpec(get_path_spec(node1),fixed=true,plan_path=false))
            set_path_spec!(node2,PathSpec(get_path_spec(node2),tight=true))
        end
    end
    set_leaf_operation_vtxs!(sched)
    process_schedule!(sched)
    sched
end

"""
    fix_precutoff_nodes!(sched::OperatingSchedule,
        problem_spec::ProblemSpec,t)

Identify all nodes that end before the cutoff time, and change their path spec
    so that the route planner will not actually plan a path for them.
"""
function fix_precutoff_nodes!(sched::OperatingSchedule,problem_spec::ProblemSpec,t)
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(sched,t)
    # Make sure no nodes are already fixed (e.g, from a previous replanning 
    # attempt that would have gone farther into the future.)
    for v in vertices(sched)
        set_path_spec!(sched,v,
            PathSpec(   
                get_path_spec(sched,v), 
                plan_path=needs_path(get_node(sched,v).node), 
                fixed=false)
                )
    end
    # set all fixed_vtxs to plan_path=false
    for v in fixed_vtxs
        set_path_spec!(sched,v,PathSpec(get_path_spec(sched,v), plan_path=false, fixed=true))
    end
    # verify that all vertices following active_vtxs have a start time > t
    @assert all(get_node(sched,v).spec.fixed==false for v in active_vtxs)
    @assert all(get_t0(sched,v) <= t for v in fixed_vtxs)
    @assert all(
        get_tF(sched,v) + minimum(get_local_slack(sched,v)) >= t for v in collect(active_vtxs)
            )
    sched
end
function fix_precutoff_nodes!(env::SearchEnv,t=minimum(map(length, get_paths(get_route_plan(env)))))
    fix_precutoff_nodes!(get_schedule(env),get_problem_spec(env),t)
    return env
end

function break_assignments!(sched::OperatingSchedule,problem_spec,v)
    n_id = get_vtx_id(sched,v)
    node = get_node_from_id(sched,n_id)
    if isa(node, AbstractRobotAction)
        new_node = replace_robot_id(node,RobotID(-1)) # All replaced ids will be filled back when propagate_valid_ids!(sched) is called again
        if isa(node,BOT_GO)
            for v2 in outneighbors(sched,v)
                if isa(get_node_from_vtx(sched,v2),Union{BOT_COLLECT,TEAM_COLLECT})
                    rem_edge!(sched,v,v2)
                    new_node = replace_destination(new_node,LocationID(-1))
                end
            end
        end
        replace_in_schedule!(sched,problem_spec,new_node,n_id)
    elseif isa(node,TEAM_ACTION)
        for i in 1:length(node.instructions)
            n = node.instructions[i]
            node.instructions[i] = replace_robot_id(node,RobotID(-1))
        end
    end
    return sched
end

"""
    break_assignments!(sched::OperatingSchedule,problem_spec::ProblemSpec)

Break all assignments that are eligible for replanning
"""
function break_assignments!(sched::OperatingSchedule,problem_spec::ProblemSpec)
    G = get_graph(sched)
    for v in vertices(G)
        path_spec = get_path_spec(sched,v)
        if path_spec.fixed == true
            continue
        end
        break_assignments!(sched,problem_spec,v)
    end
    propagate_valid_ids!(sched,problem_spec)
    sched
end

"""
    prune_schedule(sched::OperatingSchedule,
        problem_spec::ProblemSpec,t)

remove nodes that don't need to be kept around any longer
"""
function prune_schedule(sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        t,
        )
    G = get_graph(sched)

    # identify nodes "cut" by timestep
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(sched,t)
    # construct set of all nodes to prune out.
    remove_set = Set{Int}()
    for v in active_vtxs
        if isa(get_node_from_vtx(sched,v),Operation)
            push!(remove_set, v)
        end
        for e in edges(bfs_tree(G,v;dir=:in))
            if isa(get_node_from_vtx(sched, e.dst),Operation)
                push!(remove_set, e.dst)
            end
        end
    end
    for v in collect(remove_set)
        for e in edges(bfs_tree(G,v;dir=:in))
            if !(isa(get_node_from_vtx(sched, e.dst),ROBOT_AT))
                push!(remove_set, e.dst)
            end
        end
    end
    # Construct new graph
    new_sched = OperatingSchedule()
    keep_vtxs = setdiff(Set{Int}(collect(vertices(G))), remove_set)
    # add all non-deleted nodes to new project schedule
    for v in keep_vtxs
        # add_node!(new_sched,get_node(sched,v))
        add_node!(new_sched,deepcopy(get_node(sched,v)))
    end
    # add all edges between nodes that still exist
    for e in edges(get_graph(sched))
        add_edge!(new_sched, get_vtx_id(sched, e.src), get_vtx_id(sched, e.dst))
    end
    # Initialize new cache
    # draw new ROBOT_AT -> GO edges where necessary
    _robot_ids_replaced = Set{RobotID}()
    for v in vertices(new_sched)
        node = get_node(new_sched, v)
        pred = node.node
        if isa(pred,BOT_GO) && indegree(new_sched,v) == 0
            robot_id = get_robot_id(pred)
            @assert !(robot_id in _robot_ids_replaced) "Already replaced ROBOT_AT node for $(summary(robot_id))"
            push!(_robot_ids_replaced,robot_id)
            robot_node = replace_in_schedule!(new_sched, BOT_AT(robot_id, pred.x1), robot_id)
            set_t0!(robot_node,get_t0(node))
            add_edge!(new_sched, robot_node, node)
        end
        if isa(pred,Operation) && indegree(new_sched,v) < num_required_predecessors(pred)
            # add deleted objects back in
            input_ids = Set(map(v2->get_object_id(get_node_from_vtx(new_sched,v2)), inneighbors(new_sched,v)))
            for (object_id,o) in preconditions(pred)
                if !(object_id in input_ids)
                    o_node = add_node!(new_sched,make_node(sched,problem_spec,o)) 
                    add_edge!(new_sched, o_node.id, node)
                    set_t0!(o_node,get_t0(node))
                    set_tF!(o_node,get_t0(node))
                end
            end
        end
    end
    set_leaf_operation_vtxs!(new_sched)
    process_schedule!(new_sched)
    # init planning cache with the existing solution
    @assert sanity_check(new_sched)

    new_sched
end
prune_schedule(env::SearchEnv,args...) = prune_schedule(get_schedule(env),get_problem_spec(env),args...)
prune_schedule(solver,env::SearchEnv,args...) = prune_schedule(env,args...)

# """
#     `prune_project_schedule`

# Remove all vertices that have already been completed. The idea is to identify
# all `Operation`s that are completed before `t`, remove all nodes upstream of
# them (except for ROBOT_AT nodes), and create new edges between the ROBOT_AT
# nodes and their first GO assignments.
# """
# function prune_project_schedule(sched::OperatingSchedule,problem_spec::ProblemSpec,t;
#         robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
#     )
#     new_sched = prune_schedule(sched,problem_spec,t)
#     # split active nodes
#     new_sched = split_active_vtxs!(new_sched,problem_spec,t;robot_positions=robot_positions)
#     # freeze nodes that terminate before cutoff time
#     fix_precutoff_nodes!(new_sched,problem_spec,t)
#     # Remove all "assignments" from schedule
#     break_assignments!(new_sched,problem_spec)
#     new_sched
# end

"""
    splice_schedules!(sched::P,next_sched::P) where {P<:OperatingSchedule}

Merge next_sched into sched
"""
function splice_schedules!(sched::P,next_sched::P,enforce_unique=true) where {P<:OperatingSchedule}
    for v in vertices(next_sched)
        node = get_node(next_sched,v)
        if !has_vertex(sched,node)
            add_node!(sched, node)
        elseif enforce_unique
            throw(ErrorException(string("Vertex ",v," = ",
                string(node.node),
                " already in sched."
                )))
        end
    end
    for e in edges(get_graph(next_sched))
        node1 = get_node(next_sched, e.src)
        node2 = get_node(next_sched, e.dst)
        add_edge!(sched, node1, node2)
    end
    set_leaf_operation_vtxs!(sched)
    sched
end
splice_schedules!(sched,next_sched) = sched

export
    ReplannerModel,
    ReplannerConfig,
    DeferUntilCompletion,
    ReassignFreeRobots,
    MergeAndBalance,
    ConstrainedMergeAndBalance,
    Oracle,
    NullReplanner,
    get_commit_time,
    replan!

function get_timeout_buffer end
function get_route_planning_buffer end
function get_commit_time end
function break_assignments! end
function set_time_limits! end
function split_active_vtxs! end
function fix_precutoff_nodes! end
function replan! end

"""
    ReplannerModel

Abstract type. Concrete subtypes currently include `DeferUntilCompletion`,
`ReassignFreeRobots`, `MergeAndBalance`, `Oracle`, `NullReplanner`
"""
abstract type ReplannerModel end

"""
    ReplannerConfig

Stores parameters for configuring an instance of `ReplannerModel`.
Fields:
* real_time::Bool - if `true`, adjust solver runtime limits to meet real time
    operation constraints.
* timeout_buffer::Float64 - defines the minimum amount of time that must be for
    replanning the route
* route_planning_buffer::Float64 - defines the minimum amount of time that must
    be allocated for replanning the route
* commit_threshold::Float64 - defines where the ``freeze time'', or the time
    step from which the solver may change the existing plan.
"""
@with_kw mutable struct ReplannerConfig
    real_time::Bool                 = true
    timeout_buffer::Float64         = 1
    route_planning_buffer::Float64  = 2
    commit_threshold::Int           = 10
    max_time_limit::Int             = 100
    prune::Bool                     = true
end
get_replanner_config(config::ReplannerConfig) = config
get_replanner_config(model) = model.config

export
    get_real_time_flag,
    get_timeout_buffer,
    get_route_planning_buffer,
    get_commit_threshold,
    get_max_time_limit,
    set_real_time_flag!,
    set_timeout_buffer!,
    set_route_planning_buffer!,
    set_commit_threshold!,
    set_max_time_limit!

get_real_time_flag(model)           = get_replanner_config(model).real_time
get_timeout_buffer(model)           = get_replanner_config(model).timeout_buffer
get_route_planning_buffer(model)    = get_replanner_config(model).route_planning_buffer
get_commit_threshold(model)         = get_replanner_config(model).commit_threshold
get_max_time_limit(model)           = get_replanner_config(model).max_time_limit
function set_real_time_flag!(model,val) get_replanner_config(model).real_time = val end
function set_timeout_buffer!(model,val) get_replanner_config(model).timeout_buffer = val end
function set_route_planning_buffer!(model,val) get_replanner_config(model).route_planning_buffer = val end
function set_commit_threshold!(model,val) get_replanner_config(model).commit_threshold = val end
function set_max_time_limit!(model,val) get_replanner_config(model).max_time_limit = val end

"""
    DeferUntilCompletion <: ReplannerModel

Allow work to begin on the new project only after all other work is completed.
"""
@with_kw struct DeferUntilCompletion <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end
"""
    ReassignFreeRobots   <: ReplannerModel

Allow robots to begin working on the new project as soon as they have finished
their current assignments.
"""
@with_kw struct ReassignFreeRobots   <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end
"""
    MergeAndBalance      <: ReplannerModel

Allow replanning from scratch for all assignments and routes except for those
that will take place
"""
@with_kw struct MergeAndBalance      <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end
"""
    ConstrainedMergeAndBalance <: ReplannerModel

Identical to `MergeAndBalance`, except that the replanning commit time may be
pushed farther into the future to limit the total problem size at each replanning
stage.
"""
@with_kw struct ConstrainedMergeAndBalance      <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
    max_problem_size::Int   = 400 # max permissible number of unfrozen nodes in sub problem
    max_tasks::Int          = 80 # max permissible number of tasks
    constrain_tasks::Bool   = false 
    constrain_nodes::Bool   = true
end
@with_kw struct Oracle               <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig(
        timeout_buffer         = -110,
        route_planning_buffer   = 10,
        commit_threshold        = 0,
    )
end
@with_kw struct NullReplanner        <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end

export
    ReplanningProfilerCache,
    FullReplanner

"""
    ReplanningProfilerCache

Stores information during replanning
* `schedule` - maintains (non-pruned) overall project schedule
* `stage_ids` - maps id of each node in schedule to stage index
* `stage_results` - vector of result dicts
* `features` - vector of `::FeatureExtractors` to extract after each stage
* `final_features` - vector of `::FeatureExtractors` to extract after final stage
"""
@with_kw struct ReplanningProfilerCache
    schedules::Vector{OperatingSchedule}    = Vector{OperatingSchedule}()
    project_ids::Dict{AbstractID,Int}       = Dict{AbstractID,Int}()
    stage_results::Vector{Dict{String,Any}} = Vector{Dict{String,Any}}()
    final_results::Dict{String,Any}         = Dict{String,Any}()
    features::Vector{FeatureExtractor}      = Vector{FeatureExtractor}()
    final_features::Vector{FeatureExtractor}= Vector{FeatureExtractor}()
end
function reset_cache!(cache::ReplanningProfilerCache)
    empty!(cache.schedules)
    empty!(cache.project_ids)
    empty!(cache.stage_results)
    empty!(cache.final_results)
    cache
end

"""
    FullReplanner{R,S}

Wrapper for a `ReplannerModel` and a `PC_TAPF` solver, as well as a 
`ReplanningProfilerCache`.
"""
@with_kw struct FullReplanner{R,S}
    solver::S                       = NBSSolver()
    replanner::R                    = MergeAndBalance()
    cache::ReplanningProfilerCache  = ReplanningProfilerCache()
end
reset_cache!(planner::FullReplanner) = reset_cache!(planner.cache)
get_replanner_config(planner::FullReplanner) = get_replanner_config(planner.replanner)
for op in [
        :get_commit_time,
        :break_assignments!,
        :set_time_limits!,
        :split_active_vtxs!,
        :fix_precutoff_nodes!,
        ]
    @eval $op(planner::FullReplanner,args...) = $op(planner.replanner,args...)
end
CRCBS.get_logger(planner::FullReplanner) = CRCBS.get_logger(planner.solver)

export ReplannerWithBackup

"""
    ReplannerWithBackup{A,B}

Consists of a primary and a backup solver. The backup solver is supposed to be
fast--it computes a "fallback" plan in case the primary solver fails to find a
good enough solution before time out.
"""
struct ReplannerWithBackup{A,B}
    primary_planner::A
    backup_planner::B
end
for op in [:(CRCBS.set_runtime_limit!),:(CRCBS.set_deadline!)]
    @eval begin
        $op(model::FullReplanner,val) = $op(model.solver,val)
        function $op(model::ReplannerWithBackup,val)
            $op(model.backup_planner,val)
            $op(model.primary_planner,val)
        end
    end
end
# function set_runtime_limit!(model::ReplannerModel,val)
#     set_runtime_limit!(model.primary_planner,val)
#     set_runtime_limit!(model.backup_planner,val)
# end
function set_real_time_flag!(model::ReplannerWithBackup,val)
    set_real_time_flag!(model.backup_planner,val)
    set_real_time_flag!(model.primary_planner,val)
end
function reset_cache!(planner::ReplannerWithBackup)
    reset_cache!(planner.primary_planner)
    reset_cache!(planner.backup_planner)
end

function get_commit_time(replan_model, search_env, request, commit_threshold=get_commit_threshold(replan_model))
    request.t_request + commit_threshold
end
get_commit_time(replan_model::Oracle, search_env, request, args...) = request.t_request
get_commit_time(replan_model::DeferUntilCompletion, search_env, request, commit_threshold) = max(request.t_request + commit_threshold, maximum(get_tF(get_schedule(search_env))))
get_commit_time(replan_model::NullReplanner,args...) = get_commit_time(DeferUntilCompletion(),args...)
function get_earliest_free_time(search_env)
    free_time = makespan(get_schedule(search_env))
    for node in get_nodes(get_schedule(search_env))
        if isa(node.node,GO)
            if get_id(get_destination_location_id(node.node)) == -1
                free_time = min(free_time, get_t0(node))
            end
        end
    end
    return free_time
end
function get_commit_time(replan_model::ReassignFreeRobots, search_env, request, commit_threshold)
    # free_time = makespan(get_schedule(search_env))
    # for v in vertices(get_schedule(search_env))
    #     # node = get_node_from_vtx(get_schedule(search_env),v)
    #     node = get_node(get_schedule(search_env),v)
    #     if isa(node.node,GO)
    #         if get_id(get_destination_location_id(node.node)) == -1
    #             free_time = min(free_time, get_t0(node))
    #         end
    #     end
    # end
    free_time = get_earliest_free_time(search_env)
    max(request.t_request + commit_threshold,free_time)
end
function get_commit_time(replan_model::ConstrainedMergeAndBalance, search_env, request, commit_threshold)
    nv_max = max(1,replan_model.max_problem_size - nv(request.schedule))
    t_commit = request.t_request + commit_threshold
    sched = get_schedule(search_env)
    # constrain the total number of unfrozen nodes
    if replan_model.constrain_nodes
        old_t_commit = t_commit
        if 0 < nv_max < nv(sched)
            t_commit = max(t_commit, sort(get_tF(sched); rev=true)[nv_max])
        end
        if old_t_commit != t_commit
            @info "Bumping t_commit from $(old_t_commit) to $(t_commit) to respect ConstrainedMergeAndBalance.max_problem_size = $(replan_model.max_tasks)"
        end
    end
    # constrain the total number of delivery tasks
    if replan_model.constrain_tasks
        n_tasks = 0
        for n in get_nodes(request.schedule) 
            if matches_template(OBJECT_AT,n)
                n_tasks += 1
            end
        end
        for n in node_iterator(sched,reverse(sortperm(get_tF(sched))))
            if matches_template(BOT_COLLECT,n)
                n_tasks += 1
                if n_tasks > replan_model.max_tasks || get_tF(n) <= t_commit
                    old_t_commit = t_commit
                    t_commit = max(t_commit, get_tF(n))
                    if old_t_commit != t_commit
                        @info "Bumping t_commit from $(old_t_commit) to $(t_commit) to respect ConstrainedMergeAndBalance.max_tasks = $(replan_model.max_tasks)"
                    end
                    break
                end
            end
        end
    end
    return t_commit
end

function prune_schedule(::ReassignFreeRobots,env::SearchEnv,t)
    t_free = get_earliest_free_time(env)
    prune_schedule(env,min(t_free,t))
end

break_assignments!(replan_model::ReplannerModel,args...) = break_assignments!(args...)
break_assignments!(replan_model::ReassignFreeRobots,args...) = nothing
break_assignments!(replan_model::DeferUntilCompletion,args...) = nothing
break_assignments!(replan_model::NullReplanner,args...) = nothing

function set_time_limits!(replan_model,solver,t_request,t_commit)
    real_time = get_real_time_flag(replan_model)
    if real_time
        set_time_limits!(real_time,replan_model,solver,t_request,t_commit)
    end
end

function set_time_limits!(flag::Bool,replan_model,solver,t_request,t_commit)
    set_runtime_limit!(solver, (t_commit - t_request) - get_timeout_buffer(replan_model))
    set_runtime_limit!(solver, min(runtime_limit(solver), get_max_time_limit(replan_model)))
    set_runtime_limit!(assignment_solver(solver), runtime_limit(solver) - get_route_planning_buffer(replan_model))
    @assert runtime_limit(solver) > 0.0
    set_deadline!(solver, time() + (t_commit - t_request))
    solver
end
function set_time_limits!(flag::Bool,replan_model::NullReplanner,solver,t_request,t_commit)
    set_runtime_limit!(solver,-1.0)
    return solver
end

split_active_vtxs!(replan_model::ReplannerModel,new_sched,problem_spec,t_commit;kwargs...) = split_active_vtxs!(new_sched,problem_spec,t_commit;kwargs...)

fix_precutoff_nodes!(replan_model,new_sched,problem_spec,t_commit) = fix_precutoff_nodes!(new_sched,problem_spec,t_commit)


"""
    replan!(solver, replan_model, search_env, env_graph, problem_spec, route_plan,
        next_sched, t_request, t_arrival; commit_threshold=5,kwargs...)

Combine an existing solution with a new project request `next_sched` that
is requested at time `t_request`, and whose raw materials become available at
`t_arrival`.
The exact behavior of this function depends on `replan_model <: ReplannerModel`
"""
# function replan!(solver, replan_model, search_env, env_graph, problem_spec, route_plan, next_sched, t_request, t_arrival; commit_threshold=5,kwargs...)
function replan!(solver, replan_model, search_env, request;
        commit_threshold=get_commit_threshold(replan_model),
        kwargs...
        )
    # sched               = get_schedule(search_env)
    cache               = get_cache(search_env)
    route_plan          = get_route_plan(search_env)
    problem_spec        = get_problem_spec(search_env)
    env_graph           = get_graph(search_env)
    t_request           = request.t_request
    next_sched          = deepcopy(request.schedule)

    @assert sanity_check(get_schedule(search_env)," in replan!()")
    # Freeze route_plan and schedule at t_commit
    t_commit = get_commit_time(replan_model, search_env, request, commit_threshold)
    t_final = minimum(map(length, get_paths(get_route_plan(search_env))))
    @assert(all(length(p) == t_final for p in get_paths(get_route_plan(search_env))))
    t_split = min(t_commit,t_final)
    # @info "t_commit" t_commit t_split

    robot_positions=get_env_snapshot(search_env,t_split)
    reset_solver!(solver)
    set_time_limits!(replan_model,solver,t_request,t_commit)
    # Update operating schedule
    if get_replanner_config(replan_model).prune
        new_sched = prune_schedule(replan_model,search_env,t_split) # Maybe this should be at t_request instead?
    else
        new_sched = deepcopy(get_schedule(search_env))
    end
    # NOTE If we prune at t_commit or t_split, we may actually cut out unfinished
    # nodes when using ReassignFreeRobots. Ideally, ReassignFreeRobots would prune at the earliest free time
    # NOTE again -- actually, the above is not true. ReassignFreeRobots only increases t_commit to the earliest free node.
    # new_sched = prune_schedule(replan_model,search_env,min(t_split,t_request)) # Maybe this should be at t_request instead?
    @assert sanity_check(new_sched," after prune_schedule()")
    # split active nodes
    # new_sched = split_active_vtxs!(replan_model,new_sched,problem_spec,t_split;robot_positions=robot_positions)
    new_sched = split_active_vtxs!(replan_model,new_sched,problem_spec,t_commit;robot_positions=robot_positions)
    @assert sanity_check(new_sched," after split_active_vtxs!()")
    # freeze nodes that terminate before cutoff time
    # new_sched = fix_precutoff_nodes!(replan_model,new_sched,problem_spec,t_split)
    new_sched = fix_precutoff_nodes!(replan_model,new_sched,problem_spec,t_commit)
    # Remove all "assignments" from schedule
    break_assignments!(replan_model,new_sched,problem_spec)
    @assert sanity_check(new_sched," after break_assignments!()")
    # splice projects together!
    set_t0!(next_sched,t_commit) 
    splice_schedules!(new_sched,next_sched)
    process_schedule!(new_sched)
    @assert sanity_check(new_sched," after splice_schedules!()")
    base_search_env = construct_search_env(
        solver,
        new_sched,
        search_env,
        initialize_planning_cache(new_sched)
        )
    base_route_plan = initialize_route_plan(search_env,get_cost_model(base_search_env))
    trimmed_route_plan = trim_route_plan(base_search_env, base_route_plan, t_commit)
    SearchEnv(base_search_env, route_plan=trimmed_route_plan)
end
# replan!(solver, replan_model::NullReplanner, search_env, args...;kwargs...) = search_env
function replan!(solver::FullReplanner,args...;kwargs...)
    replan!(solver.solver,solver.replanner,args...;kwargs...)
end

# end
