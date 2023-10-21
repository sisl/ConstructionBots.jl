# module DFSPlanner
#
# using Parameters
# using LightGraphs
# using DataStructures
# using JLD2, FileIO
# using CRCBS
#
# using ..TaskGraphs

export
    DFSRoutePlanner

"""
    DFSRoutePlanner
Prioritized Depth-First Search route planner.
"""
@with_kw struct DFSRoutePlanner{C}
    logger::SolverLogger{C} = SolverLogger{C}()
end
# function construct_heuristic_model(solver::DFSRoutePlanner,env_graph;
#         ph = PerfectHeuristic(get_dist_matrix(env_graph)),
#         kwargs...)
#     construct_composite_heuristic(ph,ph,NullHeuristic())
# end
# function construct_cost_model(solver::DFSRoutePlanner,
#         schedule, cache, problem_spec, env_graph;
#         extra_T=400, primary_objective=SumOfMakeSpans(), kwargs...)
#     cost_model = construct_composite_cost_model(
#         typeof(primary_objective)(schedule,cache),
#         FullCostModel(maximum,TravelTime()),
#         FullCostModel(maximum,TravelDistance())
#         )
#     heuristic_model = construct_heuristic_model(solver,env_graph;kwargs...)
#     # ph = PerfectHeuristic(get_dist_matrix(env_graph))
#     # heuristic_model = construct_composite_heuristic(ph,ph,NullHeuristic())
#     cost_model, heuristic_model
# end

export
    # sorted_actions,
    get_conflict_idx,
    select_action_dfs!,
    get_next_node_matching_agent_id,
    update_envs!,
    prioritized_dfs_search

@with_kw struct DFS_SearchState
    pickup_i::Int   = 0 # if i < start, iterate from current action (inclusive). Defines where to "pick back up"
    reset_i::Int    = 0 # if i > reset_i, iterate through all possible actions. Defines the point below which to "reset" the action vector
end
function update_search_state(s,i)
    s = DFS_SearchState(s,reset_i = max(s.reset_i,s.pickup_i+1))
    if s.pickup_i > i # reset pickup_i once it has been exceeded
        s = DFS_SearchState(s, pickup_i = 0)
    end
    if s.reset_i < i
       s = DFS_SearchState(s,reset_i = i)
    end
    s
end
# function sorted_actions(env,s)
#     f = (s,a,sp)->add_heuristic_cost(env,get_transition_cost(env,s,a,sp),get_heuristic_cost(env,sp))
#     sort(
#         collect(get_possible_actions(env,s)),
#         by=a->f(s,a,get_next_state(env,s,a))
#     )
# end

"""
    get_conflict_idx(envs,states,actions,i,ordering,idxs)
Check if the planned action for agent with priority `i` conflicts with the
    planned action of any agent with higher priority.
"""
function get_conflict_idx(envs,states,actions,i,ordering,idxs)
    idx = ordering[i]
    env = envs[idx]
    s = states[idx]
    a = actions[idx]
    pi = PathNode(s,a,get_next_state(env,s,a))
    for (j,idx) in enumerate(ordering[1:max(1,i-1)])
        if j == i
            continue
        end
        env = envs[idx]
        s = states[idx]
        a = actions[idx]
        pj = PathNode(s,a,get_next_state(env,s,a))
        if detect_state_conflict(pi,pj) || detect_action_conflict(pi,pj)
            return j
        end
    end
    return -1
end
function update_envs!(solver,search_env,envs,paths)
    cache = get_cache(search_env)
    schedule = get_schedule(search_env)
    cbs_node = initialize_root_node(search_env)
    # update_planning_cache!(solver,search_env)        cache.tF[v] = get_final_state(path).t

    # update cache times
    up_to_date = true
    for i in 1:length(envs)
        env = envs[i]
        path = paths[i]
        v = get_vtx(schedule,env.node_id)
        s = get_final_state(path)
        t_arrival = max(cache.tF[v], s.t + get_distance(get_graph(search_env).dist_function,s.vtx,env.goal.vtx))
        if is_goal(envs[i],s)
            if t_arrival > cache.tF[v] && env.goal.vtx != -1
                @log_info(-1,verbosity(solver),"DFS update_envs!(): extending tF[v] from $(cache.tF[v]) to $t_arrival in ",string(get_node(env)),", s = ",string(s))
                cache.tF[v] = t_arrival
                up_to_date = false
            end
        end
    end
    # reset cache
    if !up_to_date
        t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
        cache.t0            .= t0
        cache.tF            .= tF
        cache.slack         .= slack
        cache.local_slack   .= local_slack
        # rebuild envs to reset goal time
        for i in 1:length(envs)
            env = envs[i]
            path = paths[i]
            v = get_vtx(schedule,env.node_id)
            envs[i] = build_env(solver,search_env,cbs_node,v)
        end
    end
    # mark finished envs as complete
    for i in 1:length(envs)
        env = envs[i]
        path = paths[i]
        if is_goal(envs[i],get_final_state(path))
            # update schedule and cache
            v = get_vtx(schedule,env.node_id)
            if !(v in cache.closed_set)
                update_planning_cache!(solver,search_env,v,path)
                @assert v in cache.closed_set
                @assert i == env.agent_idx
            end
        end
    end
    update_planning_cache!(solver,search_env)
    i = 0
    while i < length(envs)
        i += 1
        env = envs[i]
        path = paths[i]
        v = get_vtx(schedule,env.node_id)
        if is_goal(envs[i],get_final_state(path))
            # update schedule and cache
            if !(v in cache.closed_set)
                update_planning_cache!(solver,search_env,v,path)
                update_planning_cache!(solver,search_env)
            end
            @assert v in cache.closed_set
            @assert i == env.agent_idx
            for v2 in outneighbors(schedule,v)
                if get_path_spec(schedule,v).agent_id == i
                    if !(v2 in cache.active_set)
                        @log_info(-1,solver.l2_verbosity,"node ",string(get_node_from_vtx(schedule,v2))," not yet in active set")
                        @log_info(-1,solver.l2_verbosity,"inneighbors of ",string(get_node_from_vtx(schedule,v2))," are ",map(v3->string(get_node_from_vtx(schedule,v3)),neighborhood(schedule,v2,3,dir=:in))...)
                    end
                end
            end
            # swap out env for new env
            node_id = get_next_node_matching_agent_id(schedule,cache,env.agent_idx)
            @assert node_id != env.node_id
            if get_vtx(schedule,node_id) in cache.active_set
                envs[i] = build_env(solver,search_env,cbs_node,get_vtx(schedule,node_id))
                # i = 0
                i -= 1
            else
                node_string = string(get_node_from_id(get_schedule(search_env),node_id))
                @log_info(4,verbosity(solver),"cannot update environment for agent $i because next node ",node_string," not in cache.active_set")
            end
        end
    end
    update_cost_model!(search_env)
    envs,paths
end
function select_ordering(solver,search_env,envs)
    schedule = get_schedule(search_env)
    cache = get_cache(search_env)
    ordering = sort(
        collect(1:search_env.num_agents),
        by = i->(
            ~isa(envs[i].schedule_node,Union{COLLECT,DEPOSIT}),
            ~isa(envs[i].schedule_node,CARRY),
            minimum(cache.slack[get_vtx(schedule,envs[i].node_id)])
            )
        )
    ordering
end
function select_action_dfs!(solver,envs,states,actions,i,ordering,idxs,search_state=SearchState())
    search_state = update_search_state(search_state,i)
    if i <= 0
        return false
    elseif i > length(states)
        for (env,s,a) in zip(envs,states,actions)
            if a != CRCBS.wait(env,s) || length(get_possible_actions(env,s)) == 1 || s.t < env.goal.t
                return true
            end
        end
        @log_info(4,verbosity(solver),"action vector $(map(a->string(a),actions)) not eligible")
        return false
    elseif !(ordering[i] in idxs)
        return select_action_dfs!(solver,envs,states,actions,i+1,ordering,idxs,search_state)
    else
        # idx = env.ordering_map[i]zs
        j = 0
        idx = ordering[i]
        env = envs[idx]
        s = states[idx]
        for ai in sorted_actions(env,s)
            a = actions[idx]
            c = get_transition_cost(env,s,ai)
            c0 = get_transition_cost(env,s,a)
            if (i >= search_state.reset_i) || (i < search_state.pickup_i && a == ai) || ((c >= c0 || CRCBS.is_valid(env,a)) && a != ai)
                actions[idx] = ai
                @log_info(5,verbosity(solver),"$(repeat(" ",i))i = $i, trying a=",string(ai)," from s = ",string(s),"for env ",string(get_node(env)), " with env.goal = ",string(env.goal))
                k = get_conflict_idx(envs,states,actions,i,ordering,idxs)
                # @assert k < i "should only check for conflicts with 1:$i, but found conflict with $k"
                if k <= 0
                    if select_action_dfs!(solver,envs,states,actions,i+1,ordering,idxs,search_state)
                        return true
                    end
                elseif !(ordering[k] in idxs)
                    # i = i - 1
                    # break
                    # return false
                else
                    @log_info(5,verbosity(solver),"--- conflict betweeh $i and $k")
                    j = max(k,j)
                end
            end
        end
        if j <= 0
            return false
        end
        # if j > 0
        search_state = DFS_SearchState(pickup_i=j,reset_i=0)
        # search_states[j] = search_states[i]
        # i = j
        # break
        # end
        return select_action_dfs!(solver,envs,states,actions,j,ordering,idxs,search_state) #
    end
    # end
end
function prioritized_dfs_search(solver,search_env,envs,paths;
        t0 = max(0, minimum(map(path->length(path),paths))),
        search_state = DFS_SearchState()
        )
     tip_times = map(path->length(path),paths)
     t = t0
     states     = map(path->get_s(get_path_node(path,t+1)), paths)
     actions    = map(path->get_a(get_path_node(path,t+1)), paths)
     while true
         increment_iteration_count(solver)
         enforce_iteration_limit!(solver)
         update_envs!(solver,search_env,envs,paths)
         if length(get_cache(search_env).active_set) == 0
             return envs, paths, true
         end
         @assert all(map(p->length(p) >= t,paths))
         ordering   = select_ordering(solver,search_env,envs)
         idxs       = Set(findall(tip_times .<= t))
         if select_action_dfs!(solver,envs,states,actions,1,ordering,idxs,search_state)
             # step forward in search
             for idx in idxs
                 env = envs[idx]
                 path = paths[idx]
                 s = states[idx]
                 a = actions[idx]
                 sp = get_next_state(env,s,a)
                 push!(path,PathNode(s,a,sp))
                 set_cost!(path, accumulate_cost(env,get_cost(path),get_transition_cost(env,s,a,sp)))
             end
             t += 1
             search_state = DFS_SearchState()
             @log_info(3,verbosity(solver),"stepping forward, t = $t")
         else
             # step backward in search
             all(tip_times .> t) == 0 ? break : nothing
             for (idx,path) in enumerate(paths)
                 if tip_times[idx] < t
                     pop!(path.path_nodes)
                     @assert length(path) == t-1
                 end
             end
             t -= 1
             idxs    = Set(findall(tip_times .<= t))
              # start new search where previous search left off
             search_state = DFS_SearchState(pickup_i = maximum(idxs))
             @log_info(-1,verbosity(solver),"stepping backward, t = $t")
         end
         states     = map(path->get_s(get_path_node(path,t+1)), paths)
         actions    = map(path->get_a(get_path_node(path,t+1)), paths)
     end
     return envs, paths, false
end

"""
    Iterate over agents in order of priority, allowing them to fill in a
    reservation table for the vtxs they would like to occupy at the next time
    step(s).
"""
function CRCBS.solve!(
    solver::DFSRoutePlanner,
    mapf::P;kwargs...) where {P<:PC_MAPF}

    search_env = mapf.env
    update_planning_cache!(solver,search_env) # NOTE to get rid of all nodes that don't need planning but are in the active set

    route_plan = deepcopy(get_route_plan(search_env))
    paths = get_paths(route_plan)
    # initialize envs
    envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for p in paths])
    cbs_node = initialize_root_node(search_env)
    for i in 1:search_env.num_agents
        node_id = get_next_node_matching_agent_id(get_schedule(search_env),get_cache(search_env),i)
        envs[i] = build_env(solver,search_env,cbs_node,get_vtx(get_schedule(search_env),node_id))
    end

    envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;
        max_iters = solver.cbs_model.max_iters
    )
    if validate(get_schedule(search_env),convert_to_vertex_lists(route_plan),get_cache(search_env).t0,get_cache(search_env).tF)
        @log_info(0,verbosity(solver),"DFS: Succeeded in finding a valid route plan!")
    else
        # throw(SolverCBSMaxOutException("ERROR in DFS! Failed to find a valid route plan!"))
        # DUMP
        filename = joinpath(DEBUG_PATH,string("DFS_demo",get_debug_file_id(),".jld2"))
        mkpath(DEBUG_PATH)
        for env in envs
            v = get_vtx(get_schedule(search_env),env.node_id)
            @log_info(-1,verbosity(solver),"node ",string(get_node(env))," t0=$(get_t0(search_env,v)), tF=$(get_tF(search_env,v)), closed=$(v in get_cache(search_env).closed_set),")
        end
        robot_paths = convert_to_vertex_lists(route_plan)
        object_paths, object_intervals, object_ids, path_idxs = get_object_paths(route_plan,search_env)
        @log_info(-1,verbosity(solver),"Dumping DFS route plan to $filename")
        @save filename robot_paths object_paths object_intervals object_ids path_idxs

        throw(AssertionError("ERROR in DFS! Failed to find a valid route plan!"))
    end
    # for (path,base_path,env) in zip(paths,get_route_plan(search_env).paths,envs)
    #     c = base_path.cost
    #     for p in path.path_nodes[length(base_path)+1:end]
    #         c = accumulate_cost(env,c,get_transition_cost(env,p.s,p.a,p.sp))
    #     end
    #     path.cost = c
    # end
    cost = aggregate_costs(get_cost_model(search_env),map(p->get_cost(p),paths))

    return route_plan, get_cache(search_env), cost
end

#end