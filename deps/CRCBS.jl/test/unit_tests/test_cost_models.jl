# TravelTime
let
    model = TravelTime()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost + transition_cost
end
# TravelDistance
let
    model = TravelDistance()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost + transition_cost
end
# NullCost
let
    model = NullCost()
    cost = 1.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model,cost,transition_cost)
    @test new_cost == cost
end
# DeadlineCost
let
    t_max = 10.0
    model = DeadlineCost(t_max)
    cost = get_initial_cost(model)
    @test cost == 0.0
    transition_cost = 1.0
    new_cost = accumulate_cost(model, cost, transition_cost)
    @test new_cost == 1.0
    h_cost = 8.0
    @test add_heuristic_cost(model, cost, h_cost) == 0.0
    h_cost = 11.0
    @test add_heuristic_cost(model, cost, h_cost) == 1.0
    set_deadline!(model,0.0)
    @test add_heuristic_cost(model, cost, h_cost) == cost + h_cost
    # set_deadline!(model,t_max,t_max-4)
end
# SumOfMakeSpans cost
let
    tF = [1.0,2.0,3.0]
    root_nodes = [2,3]
    weights = [1.0,1.0]
    deadlines = [0.0,0.0]
    let
        model = SumOfMakeSpans(tF,root_nodes,weights,deadlines)
        set_deadline!(model,tF[root_nodes])
        @test model.deadlines == tF[root_nodes]

        cost = get_initial_cost(model)
        @test add_heuristic_cost(model, 0, 2) == 0 + 0
        @test add_heuristic_cost(model, 0, 3) == 1 + 0
        @test add_heuristic_cost(model, 0, 4) == 2 + 1
        for (c,g) in [(0,1),(0,2),(0,3),(0,4)]
            @test add_heuristic_cost(model, c, g) == c + sum(model.weights .* max.(0, g .- model.tF[model.root_nodes]))
        end
    end
    let
        model = MakeSpan(tF,root_nodes,weights,deadlines)
        set_deadline!(model,tF[root_nodes])
        @test model.deadlines == tF[root_nodes]
        cost = get_initial_cost(model)
        for (c,g) in [(0,1),(0,2),(0,3),(0,4)]
            @test add_heuristic_cost(model, c, g) == c + maximum(model.weights .* max.(0, g .- model.tF[model.root_nodes]))
        end
    end
end
# ConflictCostModel
let
    G = Graph(5)
    for v in 1:nv(G)-1
        add_edge!(G,v,v+1)
    end
    T = 10
    n_agents = 2
    h = HardConflictCost(G,T,n_agents)
    set_path!(h,1,[5,4,3],0)
    set_path!(h,2,[1,2,3],0)
    path_id_to_time_idx = get_conflicting_paths(h.model.table)
    @test path_id_to_time_idx[1] == path_id_to_time_idx[2] == 2
    # Agent 1 has a cost of 1 whenever it conflicts with agent 2's path
    @test get_conflict_value(h.model,1,2,1) == 1.0
    @test get_conflict_value(h.model,1,3,2) == 1.0
    @test get_conflict_value(h.model,1,4,0) == 0.0
    @test get_conflict_value(h.model,1,3,1) == 0.0
    # Agent 2 should have zero cost even when it conflicts with its own path
    @test get_conflict_value(h.model,2,2,1) == 0.0
    # Partial path reset - check that later conflicts still remain
    partially_set_path!(h.model,2,[2,1],0)
    @test get_conflict_value(h.model,1,2,1) == 0.0
    @test get_conflict_value(h.model,1,3,2) == 1.0

    set_path!(h.model,2,[3,4,5],0)
    # Now that agent 2's path has been updated, the cost should be zero for a
    # vtx no longer on the path
    @test get_conflict_value(h.model,1,3,2) == 0.0
end
# FatPaths (SoftConflictTable)
let
    for mapf in [
            CRCBS.init_fat_path_mapf(init_mapf_1(),FlatFPCost()),
            CRCBS.init_fat_path_mapf(init_mapf_1(),NormalizedFPCost()),
        ]
        table = mapf.env.cost_model.cost_models[2].model.table
        env = build_env(mapf,initialize_root_node(mapf),2)

        @test get_conflict_value(table,1,env,CBSEnv.State(5,1)) == 0
        @test get_conflict_value(table,2,env,CBSEnv.State(5,1)) >= 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(1,5),1),1) >= 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(5,1),1),1) >= 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(5,1),1),2) == 0
        cost = get_transition_cost(env,CBSEnv.State(5,0),CBSEnv.Action(Edge(5,1),1),CBSEnv.State(1,1))
        @test cost[2] >= 0.0

        CRCBS.clear_fat_path!(table,1)
        @test get_conflict_value(table,1,env,CBSEnv.State(5,1)) == 0
        @test get_conflict_value(table,2,env,CBSEnv.State(5,1)) == 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(1,5),1),1) == 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(5,1),1),1) == 0
        @test get_conflict_value(table,2,env,CBSEnv.Action(Edge(5,1),1),2) == 0
        cost = get_transition_cost(env,CBSEnv.State(5,0),CBSEnv.Action(Edge(5,1),1),CBSEnv.State(1,1))
        @test cost[2] == 0.0
    end

end
let
        mapf = CRCBS.init_fat_path_mapf(init_mapf_4(),NormalizedFPCost())
        table = mapf.env.cost_model.cost_models[2].model.table
        env = build_env(mapf,initialize_root_node(mapf),2)
        s = CBSEnv.State(5,1)
        a = CBSEnv.Action(Edge(5,1),1)

        @test get_conflict_value(table,1,env,s) == 0
        @test get_conflict_value(table,2,env,s) == 1.0/2
        @test get_conflict_value(table,2,env,reverse(a),1) == 1.0/2
        @test get_conflict_value(table,2,env,a,1) >= 1.0/2
        @test get_conflict_value(table,2,env,a,2) == 0
        s0 = CBSEnv.State(5,0)
        sp = get_next_state(env,s0,a)
        cost = get_transition_cost(env,s0,a,sp)
        @test cost[2] >= 0.0

end
# test FatPaths
let
    mapf = CRCBS.init_fat_path_mapf(CRCBS.init_mapf_4())
    solver = CBSSolver(AStar{cost_type(mapf)}())
    solution, cost = solve!(solver,mapf)
    paths = convert_to_vertex_lists(solution)
    @test paths[1] == [1,5,9,13,14,15]
    @test paths[2] == [16,12,8,4,3,2]
end
# MetaCost
let
    t0 = 0.0
    n_agents = 3
    cost = MetaCost{Float64}([1.0,3.0,4.0],0.0)
    transition_cost = [1.0,1.0,1.0]
    let
        FullCostModel(MaxCost(),FinalTime())
        FullCostModel(maximum,FinalTime())
        MakeSpan()
        SumOfTravelTime()
        SumOfTravelDistance()
    end
    let
        model = MetaCostModel(MakeSpan(),n_agents)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 5.0
    end
    let
        model = MetaCostModel(SumOfTravelTime(),n_agents)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 11.0
    end
    let
        model = FullCostModel(max,MetaCostModel(SumOfTravelTime(),n_agents))
        get_cost_model(model)
        new_cost = accumulate_cost(model,cost,transition_cost)
        @test new_cost.total_cost == 11.0
    end
    # @test MetaCost{Float64}([1.0],0.0) < MetaCost{Float64}([0.0],1.0)
end
# CompositeCost
let
    model = construct_composite_cost_model(TravelTime(),TravelTime(),TravelDistance())
    cost = (0.0,0.0,0.0)
    transition_cost = (1.0,1.0,1.0)
    @test accumulate_cost(model, cost,transition_cost) == (1.0,1.0,1.0)
    get_initial_cost(model)
    get_infeasible_cost(model)
end
let
    cost_model = construct_composite_cost_model(
        FullDeadlineCost(DeadlineCost(10)),
        FullCostModel(sum,NullCost()),
        SumOfTravelTime()
    )
    get_infeasible_cost(cost_model)
end
let
    p = Path{GraphState,GraphAction,Float64}(s0=GraphState(1,0))
    vtx_grid = initialize_dense_vtx_grid(4,4)
    graph = construct_factory_env_from_vtx_grid(vtx_grid)
    env = CBSEnv.LowLevelEnv(graph=graph)
    for e in [
            Edge(0,1),
            Edge(1,2),
            Edge(2,2),
        ]
        s = get_final_state(p)
        a = GraphAction(e,1)
        sp = get_next_state(env,s,a)
        add_to_path!(p,env,s,a,sp)
    end
    for (m,c) in [
        (TravelTime(),3),
        (TravelDistance(),2)]
    end
    @test compute_path_cost(TravelTime(),env,p,1) == 3
    @test compute_path_cost(SumOfTravelTime(),env,p,1) == 3
    @test compute_path_cost(TravelDistance(),env,p,1) == 2
    @test compute_path_cost(SumOfTravelDistance(),env,p,1) == 2
    @test compute_path_cost(construct_composite_cost_model(
        SumOfTravelTime(),
        SumOfTravelDistance(),
        ),env,p,1) == (3,2)
    @test compute_path_cost(DeadlineCost(10),env,p,1) == 3
    @test compute_path_cost(FullDeadlineCost(DeadlineCost(10)),env,p,1) == 3
end
