let
    State = TaskGraphs.State
    Action = TaskGraphs.Action

    f = pctapf_problem_1
    cost_model = SumOfMakeSpans()
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    pc_tapf = f(solver;cost_function=cost_model,verbose=false)

    base_search_env = pc_tapf.env
    prob = formulate_assignment_problem(solver.assignment_model,pc_tapf)
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
    @test validate(sched)
    search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
    pc_mapf = PC_MAPF(search_env)
    node = initialize_root_node(search_env)
    let
        env = build_env(solver,pc_mapf,search_env,node,AgentID(1))

        s = State()
        a = Action()
        get_next_state(s,a)
        get_next_state(env,s,a)

        @test State(1,2) == State(1,2)
        @test State() == State()
        @test states_match(State(),State())
        @test states_match(env,State(),State())
        CRCBS.wait(s)
        CRCBS.wait(env,s)

        get_cost_model(env)
        get_heuristic_model(env)
        @test CRCBS.is_valid(env,State()) == false
        @test CRCBS.is_valid(env,Action()) == false
        @test CRCBS.is_valid(env,State(1,2))
        @test CRCBS.is_valid(env,Action(Edge(1,2),1))

        get_heuristic_cost(env,State(1,2))
        # @test is_goal(env,State(1,0)) == false
        # @test is_goal(env,State(2,3)) == false
        # @test is_goal(env,State(1,3))
        @test length(get_possible_actions(env,State(-1,-1))) == 0
    end
    let
        for v in vertices(search_env.schedule)
            schedule_node = get_node_from_vtx(search_env.schedule,v)
            if isa(schedule_node,COLLECT)
                env = build_env(solver,pc_mapf,search_env,node,VtxID(v))
                @test length(get_possible_actions(env,State(1,0))) == 1
                @test length(get_possible_actions(env,State(-1,-1))) == 0
                break
            end
        end
    end
end
# test that conflicts are only detected between active states
let 
    for (active1,active2) in Base.Iterators.product([true,false],[true,false])
        s1 = TaskGraphs.State(1,0,active1)
        s2 = TaskGraphs.State(1,0,active2)
        a = CRCBS.wait(s1)
        n1 = PathNode(s1,a,get_next_state(s1,a))
        n2 = PathNode(s2,a,get_next_state(s2,a))
        @test detect_state_conflict(n1,n2) == (active1 && active2)
    end
end