# test SearchEnv
let
    solver = NBSSolver()
    pc_tapf = pctapf_problem_4(solver)
    pc_mapf = PC_MAPF(pc_tapf.env)

    let
        env = deepcopy(pc_mapf.env)
        cost_type(env)
        state_type(env)
        action_type(env)
        get_cost(env)
        set_solution_path!(env,get_paths(env)[1],1)
        set_path_cost!(env,get_path_costs(env)[1],1)
        set_cost!(env,typemin(cost_type(env)))
        for v in 1:nv(env.schedule)
            get_start(env,v)
        end
        @test_throws BoundsError get_start(env,nv(env.schedule)+1)

        # test that changes will not affect copies of a SearchEnv
        env2 = copy(env)
        set_cost!(env,typemax(cost_type(env)))
        @test get_cost(env2) != get_cost(env)

        path = deepcopy(get_paths(env)[1])
        extend_path!(path,4)
        set_solution_path!(env,path,1)
        @test length(get_paths(env)[1]) != length(get_paths(env2)[1])

        set_path_cost!(env,typemin(cost_type(env)),1)
        set_path_cost!(env2,typemax(cost_type(env)),1)
        @test get_path_costs(env)[1] != get_path_costs(env2)[1]

        set_t0!(env,1,get_t0(env2,1) + 1)
        set_tF!(env,1,get_tF(env2,1) + 1)
        @test get_t0(env,1) != get_t0(env2,1)
        @test get_tF(env,1) != get_tF(env2,1)
    end
    let
        node = initialize_root_node(solver,pc_mapf)
        @test solution_type(node) <: SearchEnv
        node2 = initialize_child_search_node(solver,pc_mapf,node)
        @test solution_type(node2) <: SearchEnv

        set_cost!(node,typemax(cost_type(node)))
        @test get_cost(node2) != get_cost(node)

        set_path_cost!(node,typemin(cost_type(node)),1)
        set_path_cost!(node2,typemax(cost_type(node)),1)
        @test get_path_costs(node)[1] != get_path_costs(node2)[1]

        # now reset the route plan and check that everything matches up again
        reset_route_plan!(node,node2.solution.route_plan)
        @test length(get_paths(node)[1]) == length(get_paths(node2)[1])
        @test get_path_costs(node)[1] == get_path_costs(node2)[1]
        @test get_cost(node2) == get_cost(node)

        CRCBS.cbs_update_conflict_table!(solver,pc_mapf,node2,nothing)

        set_t0!(node.solution,1,get_t0(node2.solution,1) + 1)
        set_tF!(node.solution,1,get_tF(node2.solution,1) + 1)
        @test get_t0(node.solution,1) != get_t0(node2.solution,1)
        @test get_tF(node.solution,1) != get_tF(node2.solution,1)
    end

end
# test get_env_state
let
    expected_paths = [
        [ 1,  2,  6, 10, 14, 13,  9],
        [ 4,  3,  7, 11, 15, 15, 15],
        [15, 11, 10,  9,  5,  1,  1]
    ]
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP()))
    pc_tapf = pctapf_problem_11(solver;cost_function=MakeSpan());
    c_pc_tapf = C_PC_TAPF(pc_tapf.env)
    # set_verbosity!(solver,4)
    env, cost = solve!(solver,c_pc_tapf)
    paths = convert_to_vertex_lists(get_route_plan(env))

    @test all(expected_paths .== paths)

    for (t,(preds)) in [
            (0,[OBJECT_AT(1,[2,3]), OBJECT_AT(2,11), OBJECT_AT(3,13)]), # ICs
            (1,[OBJECT_AT(1,[2,3]), OBJECT_AT(2,11), OBJECT_AT(3,13)]),
            (2,[OBJECT_AT(1,[6,7]), OBJECT_AT(2,10), OBJECT_AT(3,13)]),
            (3,[OBJECT_AT(1,[10,11]), OBJECT_AT(2,9), OBJECT_AT(3,13)]),
            (4,[OBJECT_AT(1,[14,15]), OBJECT_AT(2,5), OBJECT_AT(3,13)]),
            (5,[OBJECT_AT(1,[14,15]), OBJECT_AT(2,1), OBJECT_AT(3,13)]),
            (6,[OBJECT_AT(1,[14,15]), OBJECT_AT(2,1), OBJECT_AT(3,9)]) # Final positions
        ]
        s = get_env_state(env,t)
        for p in preds
            @test object_positions(s)[get_object_id(p)] == p
        end
    end
end
