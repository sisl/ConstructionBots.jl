# test solver initialization
let
    DefaultAStarSC()
    AStarSC{Int}()
    ISPS()
    CBSSolver()
    TaskGraphsMILPSolver()
    NBSSolver()
    solver = NBSSolver()
    best_cost(solver)
    NBSSolver(TaskGraphsMILPSolver(),CBSSolver())
end
let
    solver = NBSSolver()
    increment_iteration_count!(solver)
    increment_iteration_count!(route_planner(solver))
    @test max_iterations(solver) == 1
    @test iterations(solver) == 1
    reset_solver!(solver)
    @test max_iterations(solver) == 1
    @test iterations(solver) == 0
    hard_reset_solver!(solver)
    @test max_iterations(solver) == 0
    @test max_iterations(route_planner(solver)) == 0
    @test iterations(solver) == 0
end
# in-depth testing of solver stages on a single problem
let
    # init search env
    f = pctapf_problem_4
    cost_model = SumOfMakeSpans()
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    pc_tapf = f(solver;cost_function=cost_model,verbose=false)
    base_env = pc_tapf.env
    prob = formulate_assignment_problem(solver.assignment_model,pc_tapf)
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
    @test validate(sched)
    # Test AStarSC
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        path_planner = DefaultAStarSC()
        node = initialize_root_node(search_env)
        # plan for Robot Start
        n_id = RobotID(1)
        schedule_node = get_node(search_env.schedule, n_id)
        v = get_vtx(search_env.schedule, schedule_node)
        @test plan_path!(path_planner, pc_mapf,search_env, node, schedule_node, v)
        # plan for GO
        v2 = outneighbors(search_env.schedule,v)[1]
        schedule_node = get_node(search_env.schedule, v2)
        @test plan_path!(path_planner, pc_mapf,search_env, node, schedule_node, v2)
        # @show convert_to_vertex_lists(node.solution)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        node = initialize_root_node(search_env)
        path_planner = ISPS()
        for i in 1:nv(search_env.schedule)
            @test plan_next_path!(path_planner,pc_mapf,node.solution,node)
        end
    end
    # Test ISPS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        @test compute_route_plan!(path_planner,PC_MAPF(search_env),node)#,node)
        # @show convert_to_vertex_lists(node.solution)
        # @show convert_to_vertex_lists(get_route_plan(search_env))
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        solution, cost = solve!(path_planner,PC_MAPF(search_env))
        # @show convert_to_vertex_lists(solution.route_plan)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        @test low_level_search!(path_planner,PC_MAPF(search_env),node)
        # @show convert_to_vertex_lists(node.solution)
    end
    # Test CBS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        path_planner = CBSSolver(ISPS())
        solution, cost = solve!(path_planner,pc_mapf)
        # solution, cost = CRCBS.cbs!(path_planner,pc_mapf)
        # @show convert_to_vertex_lists(solution.route_plan)
    end
    let
        # NOTE Freezing here
        path_planner = CBSSolver(ISPS())
        env, cost = plan_route!(path_planner,deepcopy(sched),pc_tapf)
        # @show convert_to_vertex_lists(get_route_plan(env))
    end
    # test NBS
    let
        solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
        # solver = NBSSolver(assignment_model = TaskGraphsMILPSolver())
        set_iteration_limit!(solver,3)
        env, cost = solve!(solver,base_env)
        # @show convert_to_vertex_lists(get_route_plan(env))
        # @show get_logger(solver)
        # @show optimality_gap(solver) > 0
        @test validate(env.schedule)
        @test validate(env.schedule,convert_to_vertex_lists(get_route_plan(env)))
    end
end
# test task assignment
let
    # init search env
    for (i, f) in enumerate(pctapf_test_problems())
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(ExtendedAssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    # set_verbosity!(assignment_solver(solver),3)
                    # @show i, f, solver
                    pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                    search_env = pc_tapf.env

                    prob = formulate_assignment_problem(solver.assignment_model,pc_tapf)
                    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
                    # @show cost, maximum(get_tF(sched))
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost)
                    end
                    @test validate(sched)
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# test full solver
let
    # init search env
    for (i, f) in enumerate([
        pctapf_problem_1,
        pctapf_problem_2,
        pctapf_problem_3,
        pctapf_problem_4,
        pctapf_problem_5,
        pctapf_problem_6,
        pctapf_problem_7,
        pctapf_problem_8,
        ])
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(ExtendedAssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        NBSSolver(
                            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
                            path_planner = PIBTPlanner{NTuple{3,Float64}}()
                            ),
                        ]
                    # set_verbosity!(solver,3)
                    set_iteration_limit!(solver,1)
                    set_iteration_limit!(route_planner(solver),100)
                    pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                    env, cost = solve!(solver,pc_tapf)
                    # @show convert_to_vertex_lists(get_route_plan(env))
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(env.schedule)
                    @test validate(env.schedule,convert_to_vertex_lists(get_route_plan(env)))
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# Test that optimal collision-free planners obtain the correct costs
let
    cost_model = MakeSpan()
    for (i, (f,expected_cost)) in enumerate([
        (pctapf_problem_1,5),
        (pctapf_problem_2,8),
        (pctapf_problem_3,10),
        (pctapf_problem_4,3),
        (pctapf_problem_5,4),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(ExtendedAssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                env, cost = solve!(solver,pc_tapf)
                @test expected_cost == cost
            end
        end
    end
    cost_model = SumOfMakeSpans()
    for (i, (f,expected_cost)) in enumerate([
        (pctapf_problem_1,5),
        (pctapf_problem_2,8),
        (pctapf_problem_3,10),
        (pctapf_problem_4,3),
        (pctapf_problem_5,4),
        (pctapf_problem_8,16),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                env, cost = solve!(solver,pc_tapf)
                @test expected_cost == cost
            end
        end
    end
end
# verify that ISPS+A*sc avoids a collision by exploiting slack
let
    solver = NBSSolver()
    cost_model = MakeSpan()
    pc_tapf = pctapf_problem_3(solver;cost_function=cost_model,verbose=false);
    base_env = pc_tapf.env

    prob = formulate_assignment_problem(assignment_solver(solver),pc_tapf)
    project_schedule, cost = solve_assignment_problem!(assignment_solver(solver),prob,pc_tapf)
    @test termination_status(prob) == MOI.OPTIMAL

    env = construct_search_env(solver,project_schedule,base_env)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(solver,pc_mapf)
    low_level_search!(route_planner(solver),pc_mapf,node)

    path1 = convert_to_vertex_lists(node.solution.route_plan.paths[1])
    path2 = convert_to_vertex_lists(node.solution.route_plan.paths[2])
    @test path2[2] == 5 # test that robot 2 will indeed wait for robot 1
    @test path1 == [2, 6, 10, 14, 18, 22, 26, 30, 31, 32, 32]
    @test path2 == [5, 5, 6,  7,  8,  12, 12, 12, 12, 12, 16]
    # @show path1
    # @show path2
end
# Non-zero process time
let
    """
    Verify that the algorithm correctly handles scenarios where process time
    is non-zero
    """
    println("Test non-zero wait time:")
    for Δt in [0,1,4]
        let
            solver = NBSSolver()
            cost_model=MakeSpan()
            pc_tapf = pctapf_problem_6(solver;
                cost_function=cost_model,
                verbose=false,
                Δt_op=Δt
                );
            env, cost = solve!(solver,pc_tapf)
            paths = convert_to_vertex_lists(get_route_plan(env))
            # @show paths[1]
            # @show paths[2]
            @test paths[1][8+Δt] == 13
            @test paths[1][9+Δt] == 17
        end
    end
end
# Non-zero collection time
let
    """
    Verify that the algorithm correctly handles scenarios where collection time
    and deposit time are non-zero.
    """
    println("Test non-zero collection time:")
    Δt_deposit_1 = 1
    for (Δt_collect_2, true_cost) in zip([0,1,2,3,4],[6,7,8,8,8])
        solver = NBSSolver()
        cost_model = MakeSpan()
        pc_tapf = pctapf_problem_7(solver;
            cost_function=cost_model,
            verbose=false,
            Δt_op=0,
            Δt_collect=[0,Δt_collect_2,0],
            Δt_deposit=[Δt_deposit_1,0,0]
            );
        env, cost = solve!(solver,pc_tapf)
        paths = convert_to_vertex_lists(get_route_plan(env))
        @test cost == true_cost
    end
end
# Collaborative transport problems
let
    cost_model = MakeSpan()
    for (i, (f,expected_cost,expected_paths)) in enumerate([
        (pctapf_problem_11, 6, [
            [1,2,6,10,14,13,9],
            [4,3,7,11,15],
            [15,11,10,9,5,1],
        ]),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model);
                c_pc_tapf = C_PC_TAPF(pc_tapf.env)
                # set_verbosity!(solver,4)
                env, cost = solve!(solver,c_pc_tapf)
                paths = convert_to_vertex_lists(get_route_plan(env))
                @log_info(-1,verbosity(solver),sprint_route_plan(get_route_plan(env)))
                # @show paths
                @test cost == expected_cost
                for (expected_path, path) in zip(expected_paths,paths)
                    @test length(path) <= maximum(map(length,expected_paths))
                    @test path[1:length(expected_path)] == expected_path
                end
            end
        end
    end
end
# Gap repair problems
let
    solver = NBSSolver()
    set_iteration_limit!(low_level(route_planner(solver)),1)
    pc_tapf = pctapf_problem_12(solver)
    env, cost = solve!(solver,pc_tapf)
    cost
    # @show get_route_plan(env)
end
# Collaborative gap repair problems
let
    solver = NBSSolver()
    set_iteration_limit!(low_level(route_planner(solver)),1)
    pc_tapf = C_PC_TAPF(pctapf_problem_13(solver).env)
    env, cost = solve!(solver,pc_tapf)
    cost
    # @show get_route_plan(env)
end
# test ISPS backtracking
let
    solver = ISPS()
    @test TaskGraphs.activate_backtracking!(solver) == false
    @test solver.backtrack == false

    solver = ISPS(backtrack=true)
    @test solver.backtrack == true
    # activate returns false b/c no nodes to backtrack on
    @test TaskGraphs.activate_backtracking!(solver) == false
    sched = OperatingSchedule()
    add_node!(sched,GO(1,1,2),ActionID(1))
    TaskGraphs.update_backtrack_list!(solver,sched,ActionID(1))
    @test TaskGraphs.activate_backtracking!(solver) == true
    @test TaskGraphs.do_backtrack(solver,sched,ActionID(1))

end
# Test completeness of ISPS with/without backtracking
let 
    solver1 = CBSSolver(ISPS())
    prob = pctapf_problem_10(PC_MAPF,solver1)
    env,cost = solve!(solver1,prob)
    @test cost[1] == 7

    solver2 = CBSSolver(ISPS(backtrack=true))
    env,cost = solve!(solver2,prob)
    @test cost[1] == 6 

end
let 
    solver1 = CBSSolver(ISPS())
    prob = TaskGraphs.pctapf_problem_multi_backtrack(PC_MAPF,solver1)
    env,cost = solve!(solver1,prob)
    @test cost[1] == 9

    solver2 = CBSSolver(ISPS(backtrack=true))
    env,cost = solve!(solver2,prob)
    @test cost[1] == 8 

end
# Test ISPS and A* with different options 
let
    # ISPS - normal vs. no slack prioritization in queue
    # A*sc - normal vs. no collision-avoidance vs. no slack allowance
    for (i, f) in enumerate([
        pctapf_problem_1,
        pctapf_problem_2,
        pctapf_problem_3,
        pctapf_problem_4,
        pctapf_problem_5,
        pctapf_problem_6,
        pctapf_problem_7,
        pctapf_problem_8,
        ])
        for (use_slack_cost, use_conflict_cost, random_prioritization
            ) in Base.Iterators.product(
                [true,false],[true,false],[true,false]
            )
            solver = ISPS(
                low_level_planner=DefaultAStarSC(
                        use_slack_cost=use_slack_cost,
                        use_conflict_cost=use_conflict_cost
                    ),
                    random_prioritization=random_prioritization
                    )

            prob = f(PC_MAPF,solver)
            env, cost = solve!(solver,prob)
        end
    end

end
# Investigate why A*sc is failing
let 
        
end
# let
#     reset_all_id_counters!()
#     solver = NBSSolver(path_planner=CBSSolver(ISPS(backtrack=true)))
#     pc_tapf = pctapf_problem_10(solver;cost_function=MakeSpan(),verbose=false);
#     base_env = pc_tapf.env

#     prob = formulate_assignment_problem(assignment_solver(solver),pc_tapf)
#     sched, cost = solve_assignment_problem!(assignment_solver(solver),prob,pc_tapf)
#     @test cost == 6

#     env = construct_search_env(solver,sched,base_env)
#     pc_mapf = PC_MAPF(env)

#     # reset_solver!(route_planner(solver))
#     # set_verbosity!(low_level(route_planner(solver)),0)
#     # solve!(route_planner(solver),pc_mapf)

#     isps_planner = low_level(route_planner(solver))
#     reset_solver!(isps_planner)
#     TaskGraphs.clear_backtrack_list!(isps_planner)
#     set_verbosity!(isps_planner,2)
#     set_verbosity!(low_level(isps_planner),0)
#     node = initialize_root_node(solver,pc_mapf)
#     compute_route_plan!(isps_planner,pc_mapf,node)
#     search_env = node.solution

#     TaskGraphs.activate_backtracking!(isps_planner)
#     TaskGraphs.reset_schedule_times!(get_schedule(search_env),get_schedule(get_env(pc_mapf)))
#     TaskGraphs.reset_cache!(get_cache(search_env),get_schedule(search_env),)
#     TaskGraphs.reset_route_plan!(node,get_route_plan(get_env(pc_mapf)))
#     # compute_route_plan!(isps_planner,pc_mapf,node)
#     # node.solution

#     set_verbosity!(isps_planner,4)
#     while !TaskGraphs.do_backtrack(isps_planner, get_schedule(search_env), get_vtx_id(get_schedule(search_env), peek(search_env.cache.node_queue).first))
#         plan_next_path!(isps_planner,pc_mapf,search_env,node)
#     end
#     set_verbosity!(low_level(isps_planner),5)
#     plan_next_path!(isps_planner,pc_mapf,search_env,node)


# end