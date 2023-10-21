@with_kw struct MState <: CRCBS.AbstractGraphState
    vtx     ::Int           = -1
    t       ::Int           = -1
    stage   ::Int           = -1 # track more nodes than one?
    node    ::Union{Nothing,ScheduleNode}  = nothing
    active  ::Bool          = true
end
MState(s::GraphState) = MState(s.vtx,s.t)
get_stage(s::MState) = s.stage
Base.convert(::Type{MState},s::GraphState) = MState(s)
MState(vtx,t) = MState(vtx=vtx,t=t)
_state_active(s::MState) = s.active
Base.string(s::MState) = "(v=$(get_vtx(s)),t=$(get_t(s)),stage=$(get_stage(s)),node=$(summary(s.node)))"
Base.summary(s::MState) = "(v=$(get_vtx(s)),t=$(get_t(s)),stage=$(get_stage(s)),node=$(summary(s.node)))"
const MAction = CRCBS.GraphAction

@with_kw struct MPCCBSEnv{E,ID<:BotID,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{MState,MAction,C}
    search_env::E                   = nothing
    agent_id::ID                    = RobotID(-1)
    agent_idx::Int                  = get_id(agent_id)
    itineraries::Dict{BotID,Vector{ScheduleNode}} = Dict{BotID,Vector{ScheduleNode}}() # robot mission
    constraints::T                  = discrete_constraint_table(search_env,get_id(agent_id))
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_cost_model(env::MPCCBSEnv)       = env.cost_model
CRCBS.get_heuristic_model(env::MPCCBSEnv)  = env.heuristic
GraphUtils.get_graph(env::MPCCBSEnv)       = get_graph(env.search_env,graph_key(env.agent_id))
get_itinerary(env::MPCCBSEnv,id=env.agent_id) = env.itineraries[id]
for op in [:get_route_plan,:get_schedule]
    @eval $op(env::MPCCBSEnv) = $op(env.search_env)
end
for op in [
    :(CRCBS.get_paths),
    :(CRCBS.get_path_costs),
    :(CRCBS.num_agents),
    ]
    @eval $op(env::MPCCBSEnv) = $op(get_route_plan(env))
end

extract_itineraries(sched) = Dict(id=>extract_robot_itinerary(sched,id) for id in keys(get_robot_ICs(sched)))

function CRCBS.get_heuristic_cost(m::MultiStageEnvDistanceHeuristic,
    env::MPCCBSEnv,
    s::MState)
    goal_vtx = get_id(get_destination_location_id(get_itinerary(env)[get_stage(s)]))
    d = get_distance(get_graph(env),get_vtx(s),goal_vtx)
    d + CRCBS.cost_from_stage(m,get_id(get_robot_id(s.node)),get_stage(s))
end

CRCBS.EnvDeadlineCost(sched::OperatingSchedule,args...) = EnvDeadlineCost()
CRCBS.EnvDeadlineCost{E,T}(sched::OperatingSchedule,args...) where {E,T} = EnvDeadlineCost()
function CRCBS.compute_heuristic_cost(m::EnvDeadlineCost,h::MultiStageEnvDistanceHeuristic,env::MPCCBSEnv,cost,sp)
    # cost is travel time, h_cost is cost to go
    # need information about the current stage here...
    goal_vtx = get_id(get_destination_location_id(get_itinerary(env)[get_stage(sp)]))
    d = get_distance(get_graph(env),get_vtx(sp),goal_vtx)
    c = cost .+ d .- (get_tF(sp.node) .+ get_slack(sp.node))
    return m.f(max.(0.0, c))
end

construct_goal(::MPCCBSEnv,s::MState) = construct_goal(s.node)
function construct_goal(node::ScheduleNode)
    goal_idx = get_id(get_destination_location_id(node))
    goal_time = get_tF(node)
    GraphState(goal_idx,goal_time)
end
function check_stage_goal(::MPCCBSEnv,s,node=s.node)
    if get_t(s) >= get_tF(node)
        goal = construct_goal(node)
        if states_match(s, goal)
            return true
        elseif !CRCBS.is_valid(goal)
            return true
        end
    end
    return false
end
function can_advance_stage(env::MPCCBSEnv,s,node=s.node,stage=s.stage)
    if check_stage_goal(env,s,node)
        itinerary = get_itinerary(env,get_robot_id(node))
        if stage < length(itinerary)
            next_node = itinerary[stage + 1]
            # TODO wondering if something is wrong here...?
            # check position too (already taken care of by check_stage_goal...)?
            return (get_t0(next_node) <= get_t(s))
        end 
        return true
    end
    return false
end
function CRCBS.get_next_state(env::MPCCBSEnv,s::MState,a::MAction)
    stage = get_stage(s)
    node = s.node
    itinerary = get_itinerary(env,get_robot_id(node))
    sp = GraphState(get_e(a).dst,get_t(s)+get_dt(a))
    # update multiple nodes at once, if possible
    while stage < length(itinerary)
        if can_advance_stage(env,s,node,stage)
            # This should only really happen at the transition from the very first state.
            @assert matches_template(BOT_AT,s.node) "stage is $(stage), s is $(summary(s)), node is $(summary(node))"
            stage = stage + 1
            node = itinerary[stage]
        elseif can_advance_stage(env,sp,node,stage)
            stage = stage + 1
            node = itinerary[stage]
        else
            break
        end
    end
    # TODO resolve ambiguity about the completion and start times for all nodes
    MState(
        get_vtx(sp),
        get_t(sp),
        stage,
        node,
        _state_active(s),
        )
end
CRCBS.wait(::MPCCBSEnv,s)              = MAction(e=Edge(get_vtx(s),get_vtx(s)))

CRCBS.get_possible_actions(env::MPCCBSEnv,s::MState) = get_possible_actions(s.node,env,s)
function CRCBS.get_possible_actions(node,env::MPCCBSEnv,s::MState)
    if CRCBS.is_valid(env,s)
        return map(v2->MAction(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return MAction[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::MPCCBSEnv,s::MState)
    if !(get_vtx(s) == get_id(get_initial_location_id(node)))
        @log_info(-1,0,"Robot $(get_id(get_robot_id(s.node))) should not be at s=$(string(s)) when task=$(string(node))")
    end
    if CRCBS.is_valid(env,s)
        return [CRCBS.wait(env,s)]
    end
    return MAction[]
end

function CRCBS.is_goal(env::MPCCBSEnv,s)
    if get_stage(s) >= length(get_itinerary(env,get_robot_id(s.node)))
        return check_stage_goal(env,s)
    end
    return false
end

"""
    align_route_plan_tips!(env::MPCCBSEnv)   

Extend all paths to match the length of the longest path.
"""
function align_route_plan_tips!(env::MPCCBSEnv)   
    T = maximum(map(p->get_t(get_final_state(p)), get_paths(env)))
    trim_solution!(env,get_route_plan(env),T)
end

"""
    align_schedule_node_times!(env::MPCCBSEnv,sched=get_schedule(env))

Align all node completion times align with the associated robots' paths.
"""
function align_schedule_node_times!(env::MPCCBSEnv,sched=get_schedule(env))
    for (id,itinerary) in env.itineraries
        path = get_paths(env)[get_id(id)]
        # TODO: Something is wrong in here. Start and end times are not tight
        # like they ought to be.
        stage = 1
        for (idx,n) in enumerate(path.path_nodes)
            # at transitions only?
            if node_id(n.s.node) != node_id(n.sp.node)
                while stage <= length(itinerary)
                    node = itinerary[stage]
                    if can_advance_stage(env,n.s,node,stage)
                        set_tF!(node,get_t(n.s))
                        update_schedule_times!(sched,Set{Int}(get_vtx(sched,node)))
                        stage += 1
                    elseif can_advance_stage(env,n.sp,node,stage)
                        set_tF!(node,get_t(n.sp))
                        update_schedule_times!(sched,Set{Int}(get_vtx(sched,node)))
                        stage += 1
                    else
                        break
                    end
                end
            end
        end
        set_tF!(itinerary[end],get_t(get_final_state(path)))
        # stage = length(itinerary)
        # for (idx,n) in reverse(collect(enumerate(path.path_nodes)))
        #     while stage >= 1
        #         node = itinerary[stage]
        #         if can_advance_stage(env,n.s,node,stage) && !can_advance_stage(env,n.sp,node,stage) 
        #             set_tF!(node,get_t(n.s))
        #             stage -= 1
        #         # elseif can_advance_stage(env,n.sp,node,stage) 
        #         #     set_tF!(node,get_t(n.sp))
        #         #     stage -= 1
        #         else
        #             break
        #         end
        #     end
        # end
    end
    return sched
end

function CRCBS.is_consistent(env::MPCCBSEnv,prob::PC_MAPF)
    if !isempty(find_inconsistencies(env;break_on_first=true))
        return false
    end
    # check that schedule is otherwise consistent
    # return validate(env.search_env;quiet=true)
    return validate(env.search_env)
end

"""
    find_inconsistencies(env::MPCCBSEnv;

Return a dictionary mapping from node id to the amount of time by which the node
completion time exceeds the time step at which the transition occurs in the 
itinerary of the associated robot.
"""
function find_inconsistencies(env::MPCCBSEnv;
        break_on_first::Bool=false,
    )
    # stores the time step offsets for nodes in the schedule
    # Should there be any way for an inconsistency to not be detected this way?
    inconsistencies = Dict{AbstractID,Int}() 
    for path in get_paths(env)
        for n in path.path_nodes
            s = get_s(n)
            sp = get_sp(n)
            if s.node != sp.node # if path node spans a transition time
                t0 = Int(round(get_t0(sp.node)))
                tF = Int(round(get_tF(s.node)))
                if get_t(s) == 0 # first step
                    # This can also happen if the robot waits for a while...
                    # R: 0 => 0 can start and end at 0
                    # As can a string of zero duration successors, even R=>GO=>COLLECT=>CARRY=>DEPOSIT...
                    if get_t0(s.node) != get_t(s)
                        @info "node $(summary(s.node)) should have t0 == get_t(s) == $(get_t(s))"
                    end
                    if !(get_t0(sp.node) == get_t(s) || get_t0(sp.node) == get_t(sp))
                        @info "node $(summary(sp.node)) should have t0 equal to get_t(s) == $(get_t(s)) or get_t(sp) == $(get_t(sp))"
                    end
                else
                    if get_t0(sp.node) != get_t(sp)
                        @info "node $(summary(sp.node)) should have t0 == get_t(sp) == $(get_t(sp))"
                        @info "inconsistency at t0 for node $(summary(sp.node)) with n $(string(n)) path $(convert_to_vertex_lists(path))"
                        delay = t0 - get_t(sp)
                        inconsistencies[node_id(sp.node)] = delay
                    end
                    if get_tF(s.node) != get_t(sp)
                        @info "node $(summary(s.node)) should have tF == get_t(sp) == $(get_t(sp))"
                        @info "inconsistency at tF for node $(summary(sp.node)) with n $(string(n)) path $(convert_to_vertex_lists(path))"
                        delay = Int(round(get_tF(s.node) - get_t(s)))
                        inconsistencies[node_id(s.node)] = delay
                    end
                end
            end
            if (length(inconsistencies) > 0) && break_on_first
                return inconsistencies
            end
        end
    end
    # for (id,itinerary) in env.itineraries
    #     path = get_paths(env)[get_id(id)]
    #     for node in itinerary
    #         t0 = get_t0(node)
    #         v0 = get_id(get_initial_location_id(node))
    #         tF = get_tF(node)
    #         vF = get_id(get_destination_location_id(node))
    #         # check correspondence
    #         if t0 <= get_t(get_final_state(path))
    #             n0 = get_path_node(path,t0)
    #             if get_vtx(get_sp(n0)) != v0
    #                 # @info "inconsistency at t0 for node $(summary(node)) with n $(string(n0)) path $(convert_to_vertex_lists(path))"
    #                 inconsistencies[node_id(node)] = 0.0
    #             end
    #         else
    #             inconsistencies[node_id(node)] = t0 - get_t(get_final_state(path))
    #         end
    #         if tF <= get_t(get_final_state(path))
    #             nF = get_path_node(path,tF)
    #             # if !(get_vtx(get_sp(nF)) == vF || get_vtx(get_s(nF)) == vF) && valid_id(get_destination_location_id(node))
    #             if get_vtx(get_sp(nF)) != vF && valid_id(get_destination_location_id(node))
    #                 # @info "inconsistency at tF for node $(summary(node)) with and path $(convert_to_vertex_lists(path))"
    #                 inconsistencies[node_id(node)] = 0.0
    #             end
    #         else
    #             inconsistencies[node_id(node)] = 0.0
    #         end
    #     end
    # end
    return inconsistencies
end

function CRCBS.get_start(env::MPCCBSEnv,id::BotID)
    get_initial_state(get_paths(get_route_plan(env))[get_id(id)])
end


export MultiGoalPCMAPFSolver
"""
    MultiGoalPCMAPFSolver
"""
@with_kw struct MultiGoalPCMAPFSolver{C}
    low_level_planner::AStarSC{C}   = AStarSC{C}()
    logger::SolverLogger{C}         = SolverLogger{cost_type(low_level_planner)}()
end
CRCBS.low_level(m::MultiGoalPCMAPFSolver) = m.low_level_planner
MultiGoalPCMAPFSolver(s) = MultiGoalPCMAPFSolver(low_level_planner=s)
default_multi_goal_solver() = MultiGoalPCMAPFSolver(DefaultAStarSC())
search_trait(::MultiGoalPCMAPFSolver) = NonPrioritized()
CRCBS.solver_type(::MultiGoalPCMAPFSolver) = "MultiGoalPCMAPFSolver"
construct_cost_model(solver::MultiGoalPCMAPFSolver,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)

post_process_problem_type(m::NBSSolver,args...) = post_process_problem_type(m.path_planner,args...) 
post_process_problem_type(m::CBSSolver,args...) = post_process_problem_type(low_level(m),args...) 
for T in [:PC_MAPF,:PC_TAPF]
    @eval begin 
        function post_process_problem_type(m::MultiGoalPCMAPFSolver,prob::$T) 
            convert_to_multi_goal_problem($T,m,prob)
        end
    end
end

"""
    multi_goal_queue_priority(solver,env::MPCCBSEnv,id::BotID)

Compute the priority (determines the order in which paths will be computed) for 
the itinerary of robot `id`.
"""
function multi_goal_queue_priority(solver,env::MPCCBSEnv,id::BotID)
    itinerary = get_itinerary(env,id)
    slack_score = minimum(map(minimum,map(get_slack,itinerary)))
    # other ideas: sort by slack of first node
    # sort by the `n` lowest slack vals over whole itinerary  
    return slack_score
end

"""
    solve_with_multi_goal_solver!(solver,pc_mapf,

Compute a consistent solution to the PC_MAPF problem using the "multi-goal" 
appoach.
"""
function solve_with_multi_goal_solver!(solver,pc_mapf,
        base_env = build_multi_goal_env(solver,pc_mapf),
        node = initialize_root_node(solver,pc_mapf),
        ;
        start_ids::Set{BotID} = Set{BotID}(keys(base_env.itineraries)),
        )
    # TODO Is there a better way to prioritize?
    priority_queue = PriorityQueue{BotID,Float64}()
    for id in start_ids
        priority_queue[id] = multi_goal_queue_priority(solver,base_env,id)
        trim_path!(base_env,get_paths(base_env)[get_id(id)],0)
    end
    # compute all paths
    try
        while !isempty(priority_queue)
            while !isempty(priority_queue)
                id = dequeue!(priority_queue)
                @log_info(2,verbosity(solver),"Planning for $(summary(id))")
                # TODO build_env
                env = MPCCBSEnv(
                    base_env,
                    agent_id = id,
                    constraints=get_constraints(node,get_id(id)),
                    )
                base_path = get_paths(env)[get_id(id)]
                @log_info(2,verbosity(solver),"Base path = $(convert_to_vertex_lists(base_path))")
                # compute path
                reset_solver!(low_level(solver))
                path, cost = a_star!(low_level(solver),env,base_path)
                if cost == get_infeasible_cost(env)
                    @log_info(-1,verbosity(solver),"A* failed to find a feasible path. Iterations: $(iterations(low_level(solver)))")
                    return base_env
                end
                # store path in solution
                set_solution_path!(get_route_plan(env),path,get_id(id))
                set_path_cost!(get_route_plan(env),cost,get_id(id))
                # update conflict table
                # partially_set_path!(get_cost_model(env),get_id(id),convert_to_vertex_lists(path))
                set_path!(get_cost_model(env),get_id(id),convert_to_vertex_lists(path))
            end
            align_route_plan_tips!(base_env)
            # When we call align_schedule_node_times!(...) before process_schedule!, some nodes can "cheat" on can_advance_stage  
            align_schedule_node_times!(base_env)
            process_schedule!(get_schedule(base_env))

            # Check time and iterations
            enforce_time_limit!(solver)
            increment_iteration_count!(solver)
            if check_iterations(solver)
                @log_info(1,verbosity(solver), "Reached $(iteration_limit(solver))-iteration limit.")
                break
            end

            # # if inconsistent
            if !is_consistent(base_env,pc_mapf)
                inconsistencies = find_inconsistencies(base_env)
                @log_info(-1,verbosity(solver),"length(inconsistencies) = $(length(inconsistencies))")
                trim_points = select_trim_points(base_env,inconsistencies)
                @log_info(-1,verbosity(solver),"Route plan is not yet consistent. Trimming $(length(trim_points)) paths to replan")
                for (agent_id,t) in trim_points
                    path = get_paths(base_env)[get_id(agent_id)]
                    trim_path!(base_env,path,t)
                    priority_queue[agent_id] = multi_goal_queue_priority(solver,base_env,agent_id)
                end
                # reset schedule in prep for next round
                reset_schedule_times!(get_schedule(base_env),get_schedule(get_env(pc_mapf)))
                align_schedule_node_times!(base_env)
                process_schedule!(get_schedule(base_env))
            end
        end
    catch e
        isa(e, SolverException) ? handle_solver_exception(solver,e) : rethrow(e)
    end
    # TODO how to get cost?
    return base_env
end

"""
    select_trim_points(env,inconsistencies)

Choose locations at which to prune agent paths. Each path will be trimmed at the
completion time of the latest consistent node in the agent's itinerary.
"""
function select_trim_points(base_env,inconsistencies=find_inconsistencies(base_env))
    trim_points = Dict{BotID,Int}()
    # Add inconsistent paths back into priority_queue
    for (id,delay) in inconsistencies
        node = get_node(get_schedule(base_env),id)
        agent_id = get_robot_id(node)
        # Find t at which previous node began
        path = get_paths(base_env)[get_id(agent_id)]
        # inconsistencies can only occur if robot "does task too early"
        t, prev_node = backtrack_to_previous_node(path,node,get_t0(node))
        trim_points[agent_id] = t
    end
    return trim_points
end

"""
    backtrack_to_previous_node(path,node,tf)

Work backward from `tf` until finding the beginning of the previous node's path
segment.
Returns:
 * `t::Int` : the time step at which the previous node's path segment begins
 * `previous::ScheduleNode` : the previous node
"""
function backtrack_to_previous_node(path,node,tf)
    t = min(tf,length(path))
    prev_node = node
    while t > 0
        n = get_path_node(path,t)
        if node_id(get_sp(n).node) != node_id(node)
            if node_id(get_sp(n).node) != node_id(get_s(n).node)
                prev_node = get_sp(n).node
                break
            end
        end
        t -= 1
    end
    return t, prev_node
end

function build_base_multi_goal_search_env(solver,pc_mapf;
    kwargs...
    )
    # unpack
    search_env = get_env(pc_mapf)
    sched = get_schedule(search_env)
    env_graph = get_graph(search_env)
    # itineraries 
    itineraries = extract_itineraries(sched)
    vtx_sequences = Dict(get_id(k)=>map(n->get_vtx(TaskGraphs.construct_goal(n)),v) for (k,v) in itineraries)
    cost_to_go = CRCBS.construct_multi_stage_env_distance_heuristic(env_graph,vtx_sequences)
    # cost and heuristic models
    cost_model, _ = construct_cost_model(solver,search_env,EnvDeadlineCost();kwargs...)
    heuristic = construct_composite_heuristic(
        cost_to_go,
        NullHeuristic(),
        cost_to_go,
        cost_to_go,
        NullHeuristic(),
    )
    # Change route plan type in SearchEnv (Need MState instead of PCCBS.State)
    search_env = SearchEnv(
        schedule = get_schedule(search_env),
        env_layers = search_env.env_layers,
        cost_model = cost_model,
        heuristic_model = heuristic,
        num_agents = search_env.num_agents,
        route_plan = initialize_multi_goal_route_plan(sched,cost_model)
    )
end

# function preprocess_multi_goal_route_plan!(env::MPCCBSEnv)
#     route_plan = get_route_plan(env)
#     for (id,itinerary) in env.itineraries
#         path = get_paths(env)[get_id(id)]
#         start = get_initial_state(path)
#         # node = get_node(sched,k)
#         stage = start.stage
#         node = itinerary[stage]
#         while stage < length(itinerary)
#             if can_advance_stage(env,start,node,stage)
#                 stage = stage + 1
#                 node = itinerary[stage]
#             else
#                 break
#             end
#         end
#         set_solution_path!(
#             route_plan,
#             path_type(get_route_plan(env))(s0=s, cost=get_cost(path)),
#             get_id(id),
#         )
#     end
#     costs = map(p->get_cost(p), get_paths(route_plan))
#     cost = aggregate_costs(env, costs)
#     set_cost!(route_plan,cost)
#     env
# end

function build_multi_goal_env(solver,pc_mapf,
        search_env=get_env(pc_mapf),
    )
    sched = get_schedule(search_env)
    # itineraries 
    itineraries = extract_itineraries(sched)
    # base env
    env = TaskGraphs.MPCCBSEnv(
        search_env = search_env,
        itineraries = itineraries,
        )
end

function CRCBS.low_level_search!(solver::CBSSolver{L,C}, prob::PC_MAPF, node::N, idxs=collect(1:num_agents(prob))) where {C,L<:MultiGoalPCMAPFSolver,N<:ConstraintTreeNode}
    # TODO are constraints be passed correctly?
    start_ids = Set{BotID}([RobotID(i) for i in idxs])
    base_env = build_multi_goal_env(low_level(solver),prob,node.solution)
    reset_solver!(low_level(solver))
    base_env = solve_with_multi_goal_solver!(low_level(solver),prob,base_env,node;start_ids=start_ids)
    # hack to set first cost element to makespan
    cost = aggregate_costs(get_cost_model(base_env),get_path_costs(node.solution))
    fixed_cost = typeof(cost)([makespan(get_schedule(base_env)),cost[2:end]...])
    set_cost!(node, fixed_cost)
    return is_consistent(base_env,prob)
end

function convert_to_multi_goal_problem(::Type{T},solver,prob) where {T}
    search_env = build_base_multi_goal_search_env(solver,prob)
    T(search_env)
end

"""
    initialize_multi_goal_route_plan(sched::OperatingSchedule,cost_model)

Init route plan with `MState` as the state type.
"""
function initialize_multi_goal_route_plan(sched::OperatingSchedule,cost_model)
    starts = MState[]
    itineraries = extract_itineraries(sched)
    for id in sort(collect(keys(itineraries)))
        itinerary = itineraries[id]
        stage = 1
        node = itinerary[stage]
        push!(starts, MState(
            vtx = get_id(get_initial_location_id(node)),
            t = 0, 
            stage = stage,
            node = node,
            ))
    end
    paths = map(s->Path{MState,Action,cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end


################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

function CRCBS.initialize_root_node(solver::CBSSolver{L,C},pc_mapf::AbstractPC_MAPF) where {L<:MultiGoalPCMAPFSolver,C}
    env = build_base_multi_goal_search_env(solver,pc_mapf)
    solution = env
    # solution = build_base_multi_goal_env(solver,pc_mapf)
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            i=>discrete_constraint_table(env,i) for i in 1:num_agents(env)
            ),
        id = 1)
end