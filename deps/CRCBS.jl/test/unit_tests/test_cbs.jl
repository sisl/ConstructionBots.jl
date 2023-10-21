let
    s = CBSEnv.State()
    @test states_match(s,s)
    a = CBSEnv.Action()
    env = CBSEnv.LowLevelEnv()

    a = CRCBS.wait(s)
    sp = get_next_state(s,a)
    sp = get_next_state(env,s,a)
    @test states_match(s,sp)
    @test is_goal(env,s)
end
let
    s = CBSEnv.State()
    a = CBSEnv.Action()
    env = CBSEnv.LowLevelEnv()
    get_next_state(s,a)
    get_transition_cost(env,s,a,s)
end
let
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = CBSEnv.LowLevelEnv(graph=G)
    for a in get_possible_actions(env,CBSEnv.State(2,0))
        @test a.e.src == 2
        @test a.e.dst ∈ [1,2,3]
    end
end
let
    CBSEnv.LowLevelEnv(cost_model=SumOfTravelTime())
end
let
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    T = 10
    n_agents = 2
    env = CBSEnv.LowLevelEnv(graph=G,agent_idx=1,cost_model=HardConflictCost(G,T,n_agents))
    set_path!(get_cost_model(env),2,[2,6,10,14])
    @test get_transition_cost(env,CBSEnv.State(5,0),CBSEnv.Action(Edge(5,6),1),CBSEnv.State(6,1)) == 1
end
# solve!
let
    S = CBSEnv.State
    A = CBSEnv.Action
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=1,n_obstacles_y=1)
    #  1   2   3   4   5   6
    #  7   8   9  10  11  12
    # 13  14   0   0  15  16
    # 17  18   0   0  19  20
    # 21  22  23  24  25  26
    # 27  28  29  30  31  32
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [CBSEnv.State(1,0),CBSEnv.State(2,0)]
    goals = [CBSEnv.State(vtx=6),CBSEnv.State(vtx=5)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = CBSEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    solver = CBSSolver()
    initialize_root_node(solver,mapf)
    default_solution(mapf)
    # println("Testing CBS")
    # low_level_search
    let
        solver = CBSSolver()
        node = CBSEnv.initialize_root_node(mapf)
        initialize_child_search_node(solver,mapf,node)
        low_level_search!(solver,mapf,node)
    end
    # high_level_search
    let
        solver = CBSSolver()
        set_iteration_limit!(solver,0)
        # @test_throws SolverException CRCBS.solve!(solver,mapf)
    end
    let
        solver = CBSSolver()
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,100)
        solution, cost = CRCBS.solve!(solver,mapf)
        @test cost == 10
    end
    # with CompositeCost and CompositeHeuristic
    let
        cost_model = construct_composite_cost_model(
            FullCostModel(sum,NullCost()),
            FullCostModel(sum,TravelTime())
        )
        heuristic = construct_composite_heuristic(
            NullHeuristic(),
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
        )
        env = CBSEnv.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        solution, cost = CRCBS.solve!(solver,mapf)
    end
    let
        deadline = ne(G)+1.0
        cost_model = construct_composite_cost_model(
            FullDeadlineCost(DeadlineCost(deadline)),
            FullCostModel(sum,NullCost()),
            SumOfTravelTime()
        )
        heuristic_model = construct_composite_heuristic(
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals)),
            HardConflictHeuristic(G,ne(G),num_agents(mapf)),
            PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals)),
        )
        env = CBSEnv.LowLevelEnv(graph=G,cost_model=cost_model,heuristic=heuristic_model)
        mapf = MAPF(env,starts,goals)
        default_solution(mapf)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        solution, cost = CRCBS.solve!(solver,mapf)
    end
end
# problem instances
let
    solver = CBSSolver()
    set_iteration_limit!(solver,1000)
    for f in [init_mapf_1,init_mapf_2,init_mapf_4,CRCBS.init_mapf_8]
        # @info "$(f)"
        prob = f()
        reset_solver!(solver)
        solution, cost = solve!(solver,prob)
    end

end
# test copying
let
    solver = CBSSolver()
    prob = init_mapf_1()
    solution, cost = solve!(solver,prob)
    solution2 = copy(solution)
    # should not affect solution2
    set_solution_path!(solution,get_paths(solution)[1],2)
    @test convert_to_vertex_lists(solution)[2] != convert_to_vertex_lists(solution2)[2]
    n = get_paths(solution)[1].path_nodes[end]
    # should affect both solutions
    push!(get_paths(solution)[1],n)
    @test length(get_paths(solution)[1]) == length(get_paths(solution2)[1])

end
