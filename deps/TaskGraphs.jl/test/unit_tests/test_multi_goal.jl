using TaskGraphs
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)
using Test
using LightGraphs

let
    for f in [
            pctapf_problem_1,
            pctapf_problem_2,
            pctapf_problem_3,
            pctapf_problem_4,
            pctapf_problem_5,
            pctapf_problem_6,
            pctapf_problem_7,
            pctapf_problem_8,
            pctapf_problem_9,
            pctapf_problem_10,
        ]
        @show f
        solver = NBSSolver()
        prob = f(solver)
        solution,cost = solve!(solver,prob)
        pcta = PC_TA(prob.env)
        sched, cost = solve!(solver.assignment_model,pcta)
        pc_mapf = construct_routing_problem(prob,construct_search_env(solver,sched,get_env(prob)))

        solver = TaskGraphs.MultiGoalPCMAPFSolver(DefaultAStarSC())
        pc_mapf_mod = TaskGraphs.convert_to_multi_goal_problem(PC_MAPF,
            low_level(solver),
            pc_mapf)
        env = TaskGraphs.solve_with_multi_goal_solver!(solver,pc_mapf_mod)
        for (id,itinerary) in env.itineraries
            for (n1,n2) in zip(itinerary,itinerary[2:end])
                @test get_tF(n1) == get_t0(n2)
            end
        end
        env.search_env
    end

end
# solve everything
let
    for f in [
            pctapf_problem_1,
            pctapf_problem_2,
            pctapf_problem_3,
            pctapf_problem_4,
            pctapf_problem_5,
            pctapf_problem_6,
            pctapf_problem_7,
            pctapf_problem_8,
            pctapf_problem_9,
            pctapf_problem_10,
        ]
        @show f
        solver = NBSSolver(path_planner=CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC())))
        prob = f(solver)
        env, cost = solve!(solver,prob)
        @test validate(env)

        let 
            # test align_schedule_node_times
            mg_env = TaskGraphs.build_multi_goal_env(solver,prob,deepcopy(env))
            @test is_consistent(mg_env,PC_MAPF(get_env(prob)))
            # now "unalign" schedule times and realign them
            sched = get_schedule(mg_env)
            t0 = deepcopy(get_t0(sched))
            tF = deepcopy(get_tF(sched))
            # reset schedule
            TaskGraphs.reset_schedule_times!(sched,get_schedule(get_env(prob)))
            # set_t0!(sched,0.0)
            # set_tF!(sched,0.0)
            # TaskGraphs.update_schedule_times!(sched)
            TaskGraphs.align_schedule_node_times!(mg_env)
            # TaskGraphs.update_schedule_times!(sched)
            # TaskGraphs.align_schedule_node_times!(mg_env)
            TaskGraphs.update_schedule_times!(sched)
            t0_err = Int.(round.(t0 .- get_t0(sched)))
            tF_err = Int.(round.(tF .- get_tF(sched)))
            err_set = findall((t0_err .!= 0) .| (tF_err .!= 0))
            if !isempty(err_set)
                for v in err_set
                    node = get_node(sched,v)
                    @show v, t0[v], tF[v], summary(node)
                end
                @show get_route_plan(mg_env)
            end
            @test is_consistent(mg_env,PC_MAPF(get_env(prob)))
        end
    end

end

# test align_schedule_node_times
let 

end

# Test case designed to challenge the multi-goal solver
let

end

let 
    # solver = NBSSolver(path_planner=CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC())))
    # set_verbosity!(solver.path_planner,3)
    # set_verbosity!(solver.path_planner.low_level_planner,3)
    # set_verbosity!(solver.path_planner.low_level_planner.low_level_planner,5)
    # set_iteration_limit!(solver.path_planner.low_level_planner,2)
    # set_iteration_limit!(solver.path_planner,1)
    # prob = TaskGraphs.convert_to_multi_goal_problem(PC_TAPF,solver,pctapf_problem_4(solver))
    # prob = pctapf_problem_4(solver)
    solver = CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC()))
    prob = TaskGraphs.generate_prob(PC_MAPF,TaskGraphs.pctapf_problem_15,solver)
    node = initialize_root_node(solver,prob)
    low_level_search!(solver,prob,node)
    # add_constraint!(node,)
    env, cost = solve!(solver,prob)
    for p in get_paths(get_route_plan(env))
        for t in 0:get_t(get_final_state(p))
            println(string(get_path_node(p,t)))
        end
        println("")
    end

    # n = get_path_node(get_paths(env)[1],1)
    # c = state_constraint(1,n,get_t(get_sp(n)))
    # node = initialize_root_node(solver,prob)
    # add_constraint!(env,node,c)
    # @show low_level_search!(solver,prob,node)
    # node.solution

end

let
    solver = CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC()))
    prob = TaskGraphs.generate_prob(PC_MAPF,TaskGraphs.pctapf_problem_1,solver)
    # node = initialize_root_node(solver,prob)
    # low_level_search!(solver,prob,node)
    # add_constraint!(node,)
    env, cost = solve!(solver,prob)
    for p in get_paths(get_route_plan(env))
        for t in 0:get_t(get_final_state(p))
            println(string(get_path_node(p,t)))
        end
        println("")
    end
    mg_env = TaskGraphs.build_multi_goal_env(solver,prob,deepcopy(env))
    itineraries = TaskGraphs.extract_itineraries(get_schedule(mg_env))
    collect_node = itineraries[RobotID(1)][3]
    set_t0!(collect_node,get_t0(collect_node)-1)
    TaskGraphs.find_inconsistencies(mg_env)

end

let 
    solver = NBSSolver(path_planner=CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC())))
    # set_verbosity!(solver.path_planner,3)
    prob = pctapf_problem_4(solver)
    solution, results = profile_solver!(solver,prob)
    
end

let
    solver = NBSSolver()
    prob = pctapf_problem_3(NBSSolver())
    solution, cost = solve!(solver,prob)

    # pcta = PC_TA(prob.env)
    # sched, cost = solve!(solver.assignment_model,pcta)
    pc_mapf = construct_routing_problem(prob,construct_search_env(solver,sched,get_env(prob)))

    # solver = CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC()))
    solver = NBSSolver(path_planner=CBSSolver(MultiGoalPCMAPFSolver(DefaultAStarSC())))
    prob = TaskGraphs.convert_to_multi_goal_problem(PC_MAPF,solver,pc_mapf)

    solve!(solver.path_planner,prob)

    node = initialize_root_node(solver,prob)
    low_level_search!(solver.path_planner,prob,node)
    detect_conflicts!(node)

    env = TaskGraphs.solve_with_multi_goal_solver!(solver,pc_mapf);

    for (id,itinerary) in env.itineraries
        for n in get_paths(env)[get_id(id)].path_nodes
            @show string(n.s), string(n.sp)
        end
    end

    base_env = TaskGraphs.build_base_multi_goal_env(solver,pc_mapf) 

    agent_id = RobotID(1)
    env = TaskGraphs.MPCCBSEnv(base_env,agent_id = agent_id,agent_idx=get_id(agent_id))
    s = get_start(env,agent_id)

    @test TaskGraphs.check_stage_goal(env,s)
    @test TaskGraphs.check_stage_goal(env,s)

    c = get_initial_cost(env)
    @test c[1] == 0.0
    h = CRCBS.compute_heuristic_cost(env,c,s)
    @test h[1] == 0.0

    a = CRCBS.wait(env,s)
    sp = get_next_state(env,s,a)
    @test !TaskGraphs.check_stage_goal(env,sp)
    c = get_transition_cost(env,s,a,sp)
    @test c[1] == 1
    h = CRCBS.compute_heuristic_cost(env,c,sp)
    @test h[1] == 1

    a = GraphAction(Edge(s.vtx,5),1)
    sp = get_next_state(env,s,a)
    c = get_transition_cost(env,s,a,sp)
    @test c[1] == 1
    h = CRCBS.compute_heuristic_cost(env,c,sp)
    @test h[1] == 0
end