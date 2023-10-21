let
    MultiStageCBS.State()
    MultiStageCBS.Action()
    MultiStageCBS.LowLevelEnv()
    s = MultiStageCBS.State(vtx=1)
    @test states_match(s,s)
    a = MultiStageCBS.Action()
    env = MultiStageCBS.LowLevelEnv()
    a = CRCBS.wait(s)
end
let
    G = Graph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,2)
    add_edge!(G,2,3)
    env = MultiStageCBS.LowLevelEnv(graph=G)
    for a in get_possible_actions(env,MultiStageCBS.State(vtx=2))
        @test get_e(a).src == 2
        @test get_e(a).dst ∈ [1,2,3]
    end
end
let
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=1,n_obstacles_y=1)
    #  1   2   3   4   5   6
    #  7   8   9  10  11  12
    # 13  14   0   0  15  16
    # 17  18   0   0  19  20
    # 21  22  23  24  25  26
    # 27  28  29  30  31  32
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [
        MultiStageCBS.State(vtx=1,stage=1,t=0),
        MultiStageCBS.State(vtx=2,stage=1,t=0),
        ]
    goals = [
        [MultiStageCBS.State(vtx=27),MultiStageCBS.State(vtx=13)],
        [MultiStageCBS.State(vtx=28),MultiStageCBS.State(vtx=14)],
        ]
    let
        heuristic = MultiStagePerfectHeuristic(
            G, map(g->map(s->s.vtx, g), goals)
        )
        env = MultiStageCBS.LowLevelEnv(graph=G,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,10)
        node = initialize_root_node(mapf)
        @test low_level_search!(solver,mapf,node)
        # convert_to_vertex_lists(node.solution)
    end
    let
        heuristic = MultiStagePerfectHeuristic(
            G, map(g->map(s->s.vtx, g), goals)
        )
        env = MultiStageCBS.LowLevelEnv(graph=G,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,10)
        node = initialize_root_node(mapf)
        solution, cost = CRCBS.solve!(solver,mapf)
        # @test cost == 16
    end
    let
        env = MultiStageCBS.LowLevelEnv(
            graph=G,
            cost_model = construct_composite_cost_model(
                FullCostModel(sum,NullCost()),
                FullCostModel(sum,TravelTime())
                ),
            heuristic = construct_composite_heuristic(
                NullHeuristic(),
                MultiStagePerfectHeuristic(G, map(g->map(s->s.vtx, g), goals))
                )
            )
        mapf = MAPF(env,starts,goals)
        node = initialize_root_node(mapf)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,10)
        solution, cost = CRCBS.solve!(solver,mapf);
        # @test cost[2] == 16
    end
end
let
    # solver = MultiStageCBS.CBSSolver()
    vtx_grid = initialize_regular_vtx_grid(;n_obstacles_x=2,n_obstacles_y=2,obs_offset = [1;1])
    #  1   2   3   4   5   6   7   8   9  10
    # 11  12  13  14  15  16  17  18  19  20
    # 21  22   0   0  23  24   0   0  25  26
    # 27  28   0   0  29  30   0   0  31  32
    # 33  34  35  36  37  38  39  40  41  42
    # 43  44  45  46  47  48  49  50  51  52
    # 53  54   0   0  55  56   0   0  57  58
    # 59  60   0   0  61  62   0   0  63  64
    # 65  66  67  68  69  70  71  72  73  74
    # 75  76  77  78  79  80  81  82  83  84
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [
        MultiStageCBS.State(vtx=1,stage=1,t=0),
        MultiStageCBS.State(vtx=2,stage=1,t=0),
        # MultiStageCBS.State(vtx=26,stage=1,t=10),
        # MultiStageCBS.State(vtx=19,stage=1,t=10),
        # MultiStageCBS.State(vtx=41,stage=1,t=4)
        ]
    goals = [
        [MultiStageCBS.State(vtx=14),MultiStageCBS.State(vtx=81)],
        [MultiStageCBS.State(vtx=5),MultiStageCBS.State(vtx=55)],
        # [MultiStageCBS.State(vtx=60),MultiStageCBS.State(vtx=52)],
        # [MultiStageCBS.State(vtx=61)],
        # [MultiStageCBS.State(vtx=32),MultiStageCBS.State(vtx=1)]
        ]
    let
        heuristic = MultiStagePerfectHeuristic(
            G, map(g->map(s->s.vtx, g), goals)
        )
        env = MultiStageCBS.LowLevelEnv(graph=G,heuristic=heuristic)
        mapf = MAPF(env,starts,goals)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,10)
        node = initialize_root_node(mapf)
        solution, cost = CRCBS.solve!(solver,mapf)
    end
    let
        env = MultiStageCBS.LowLevelEnv(
            graph=G,
            cost_model = construct_composite_cost_model(
                FullCostModel(sum,NullCost()),
                FullCostModel(sum,TravelTime())
                ),
            heuristic = construct_composite_heuristic(
                NullHeuristic(),
                MultiStagePerfectHeuristic(G, map(g->map(s->s.vtx, g), goals))
                )
            )
        mapf = MAPF(env,starts,goals)
        node = initialize_root_node(mapf)
        solver = CBSSolver(AStar{cost_type(mapf)}())
        set_verbosity!(solver,global_verbosity())
        set_iteration_limit!(solver,10)
        solution, cost = CRCBS.solve!(solver,mapf);
    end
end
