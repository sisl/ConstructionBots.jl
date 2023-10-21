let
    cost_model = SumOfMakeSpans()
    for (i, f) in enumerate(pctapf_test_problems())
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}()
            )
        let
            pc_tapf = f(solver;cost_function=cost_model,verbose=false)
            base_search_env = pc_tapf.env
            prob = formulate_assignment_problem(solver.assignment_model,pc_tapf)
            sched, _ = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)

            pibt_planner = route_planner(solver)
            search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
            pc_mapf = PC_MAPF(search_env)
            set_iteration_limit!(pibt_planner,50)
            set_verbosity!(pibt_planner,0)
            solution, valid_flag = pibt!(pibt_planner,deepcopy(pc_mapf))
            # @show i, f, get_cost(solution)
            # @show solution.route_plan
            @test valid_flag
            @test validate(solution) 

            reset_solver!(solver)
            solution, cost = solve!(pibt_planner,pc_mapf)
            @test cost != typemax(cost_type(pc_mapf))
        end
    end
end
# Test that PIBT can terminate early with a partial plan that is viable
let
    solver = NBSSolver(
        assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
        path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true)
        )
    set_iteration_limit!(solver,1)
    set_iteration_limit!(solver.path_planner,5)
    set_deadline!(solver,Inf)
    set_runtime_limit!(solver,Inf)
    rpctapf = replanning_problem_1(solver)
    replan_model = MergeAndBalance()

    base_env = deepcopy(rpctapf.env)
    env = replan!(solver,replan_model,base_env,rpctapf.requests[1])
    pc_tapf = PC_TAPF(env)

    reset_solver!(solver)
    solution, cost = solve!(solver,pc_tapf)

    fix_precutoff_nodes!(solution)
    reset_solver!(solver)
    set_iteration_limit!(solver.path_planner,100)
    solution2, cost = solve!(solver,PC_TAPF(solution))

    for v in solution2.cache.active_set
        node = get_node_from_vtx(solution2.schedule,v)
        @test isa(node,GO)
        @test get_destination_location_id(node) == LocationID(-1)
    end

    # prob = formulate_assignment_problem(solver.assignment_model,pc_tapf)
    # sched, _ = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
    #
    # pibt_planner = route_planner(solver)
    # pc_mapf = PC_MAPF(construct_search_env(solver,deepcopy(sched),pc_tapf.env))
    # set_iteration_limit!(pibt_planner,5)
    # set_verbosity!(pibt_planner,0)
    # solution, valid_flag = pibt!(pibt_planner,pc_mapf);
    #
    # remap_object_ids!(rpctapf.requests[2].schedule,solution.schedule)
    # env = replan!(solver,replan_model,solution,rpctapf.requests[2])
    # # update_planning_cache!(solver,env)
    # reset_solver!(solver)
    # solution2, cost = solve!(solver,PC_TAPF(env))

end