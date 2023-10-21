CRCBS.build_env(solver,pc_mapf::AbstractPC_MAPF,env::SearchEnv,node::ConstraintTreeNode,i::Int) = build_env(solver,pc_mapf,env,node,AgentID(i))
function construct_heuristic_model(solver::PIBTPlanner,env_graph;
        ph = PerfectHeuristic(get_dist_matrix(env_graph)),
        kwargs...)
    construct_composite_heuristic(ph,ph,NullHeuristic())
end
function construct_cost_model(solver::PIBTPlanner,
        schedule, cache, problem_spec, env_graph, primary_objective=SumOfMakeSpans(); kwargs...)
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(schedule,cache),
        FullCostModel(maximum,TravelTime()),
        FullCostModel(maximum,TravelDistance())
        )
    heuristic_model = construct_heuristic_model(solver,env_graph;kwargs...)
    cost_model, heuristic_model
end

function CRCBS.is_consistent(cache::PIBTCache,pc_mapf::PC_MAPF)
    search_env = CRCBS.get_solution(cache)
    n_jobless = 0
    # for e in CRCBS.get_envs(cache)
    #     if !CRCBS.is_valid(get_goal(e))
    #         n_jobless += 1
    #     end
    # end
    if length(get_cache(search_env).closed_set) + n_jobless >= nv(get_schedule(search_env))
        return true
    elseif !any(CRCBS.is_valid, map(e->CRCBS.get_goal(e),CRCBS.get_envs(cache)))
        # all robots are done with their jobs
        return true
    end
    return false
end

function CRCBS.pibt_priority_law(solver,pc_mapf::PC_MAPF,cache,i)
    env = CRCBS.get_envs(cache)[i]
    return (
        ~CRCBS.is_valid(env,get_goal(env)),
        # ~isa(get_node(env),DEPOSIT),~isa(get_node(env),CARRY),~isa(get_node(env),COLLECT),
        ~isa(get_node(env),DEPOSIT),~isa(get_node(env),COLLECT),~isa(get_node(env),CARRY),
        minimum(get_slack(cache.solution,env.node_id)),
        -CRCBS.get_timers(cache)[i],
        i
    )
end

function CRCBS.pibt_preprocess!(solver,pc_mapf::PC_MAPF,cache)
    update_planning_cache!(solver,CRCBS.get_solution(cache))
    CRCBS.pibt_update_envs!(solver,pc_mapf,cache)
end

"""
    CRCBS.pibt_update_solution!(solver,pc_mapf::PC_MAPF,solution::SearchEnv,cache)

Overridden to update the SearchEnv
"""
function CRCBS.pibt_update_solution!(solver,solution::SearchEnv,cache)
    # update route plan
    CRCBS.pibt_update_solution!(solver,get_route_plan(solution),cache)
    # NOTE Update planning cache so that all goal times can be updated
    update_planning_cache!(solver,solution)
    solution
end
function pibt_info_string(cache,i,
        env=CRCBS.get_envs(cache)[i],
        s=CRCBS.get_states(cache)[i],
        a=CRCBS.get_actions(cache)[i]
    )
    string("\t",i,": ", get_vtx(cache.solution.schedule,env.node_id)," ",
        string(get_node(env))," s=",string(s),", a=",string(a),
        ", goal=",string(env.goal),"\n")
end
function pibt_info_strings(cache)
    info_strings = String[]
    for i in CRCBS.get_ordering(cache)
        str = pibt_info_string(cache,i)
        push!(info_strings,str)
    end
    return info_strings
end
function CRCBS.pibt_update_envs!(solver,pc_mapf::PC_MAPF,cache)
    @log_info(2,verbosity(solver),"PIBT iteration $(iterations(solver)) PRE UPDATE:\n",
        pibt_info_strings(cache)...)
    solution = CRCBS.get_solution(cache)
    node = initialize_root_node(solver,pc_mapf)
    done = true
    sweeps = 0
    while true
        done = true
        for (i,p) in enumerate(get_paths(solution))
            # env = CRCBS.get_envs(cache)[i]
            # v = get_vtx(solution.schedule,env.node_id)
            # sp = get_final_state(p)
            v_next = get_next_vtx_matching_agent_id(solution,RobotID(i))
            # NOTE: Adding the "is_goal" condition below sometimes causes PIBT 
            # to allow robots to begin DEPOSIT tasks before they are in place. 
            # No idea why, but for now: DO NOT TOUCH!
            if has_vertex(solution.schedule,v_next) #&& is_goal(env,sp) 
                # rebuild env to ensure that the goal time is up to date
                CRCBS.get_envs(cache)[i] = build_env(solver,pc_mapf,solution,node,AgentID(i))
                # env = CRCBS.get_envs(cache)[i]
            end
            env = CRCBS.get_envs(cache)[i]
            v = get_vtx(solution.schedule,env.node_id)
            sp = get_final_state(p)
            if (v in solution.cache.active_set) && is_goal(env,sp) && CRCBS.is_valid(env,get_goal(env))
                done = false
                update_env!(solver,solution,v,p)
                update_planning_cache!(solver,solution)
                v_next = get_next_vtx_matching_agent_id(solution,RobotID(i))
                @assert v_next != v
                if has_vertex(solution.schedule,v_next)
                    CRCBS.get_timers(cache)[i] = 0
                end
            end
        end
        sweeps += 1
        done ? break : nothing
    end
    @log_info(2,verbosity(solver),"PIBT iteration $(iterations(solver)) POST UPDATE:\n",
        pibt_info_strings(cache)...)
    return cache
end
