let
    S = CBSEnv.State
    A = CBSEnv.Action
    P = PathNode{S,A}
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    starts = [CBSEnv.State(1,0),CBSEnv.State(2,0)]
    goals = [CBSEnv.State(vtx=5),CBSEnv.State(vtx=6)]
    heuristic = PerfectHeuristic(G,map(s->s.vtx,starts),map(s->s.vtx,goals))
    env = CBSEnv.LowLevelEnv(graph=G,heuristic=heuristic)
    mapf = MAPF(env,starts,goals)
    let
        root_node = initialize_root_node(mapf)
        paths = [
            Path([P(S(1,1),A(Edge(1,2),2),S(2,3))]),
            Path([P(S(2,1),A(Edge(2,1),2),S(1,3))]),
        ]
        table = detect_conflicts(paths)
        @test CRCBS.is_action_conflict(get_next_conflict(table))
        set_solution_path!(root_node.solution,paths[1],1)
        set_solution_path!(root_node.solution,paths[2],2)
        detect_conflicts!(root_node)
        @test count_conflicts(root_node) == 1
        @test count_conflicts(root_node,1,2) == count_conflicts(root_node,2,1)
        @test count_conflicts(root_node,[1,2],[1,2]) == 1
        c = get_next_conflict(root_node)
        constraints = generate_constraints_from_conflict(c)
        @test length(constraints) == 2
        for c in constraints
            @test is_action_constraint(c)
        end
        add_constraint!(env,root_node,constraints[1])
    end
    let
        root_node = initialize_root_node(mapf)
        paths = [
            Path([P(S(1,0),A(Edge(1,2),0),S(2,1))]),
            Path([P(S(2,0),A(Edge(2,2),0),S(2,1))]),
        ]
        set_solution_path!(root_node.solution,paths[1],1)
        set_solution_path!(root_node.solution,paths[2],2)
        child1 = initialize_child_search_node(root_node)
        child2 = initialize_child_search_node(root_node)
        detect_conflicts!(child1,[1])
        @test count_conflicts(child1) == 1
        @test count_conflicts(child1,1,2) == 1
        @test count_conflicts(child1,2,1) == 1
        @test count_conflicts(child1,[1,2],[1,2]) == 1

        c = get_next_conflict(child1)
        constraints = generate_constraints_from_conflict(c)
        @test length(constraints) == 2
        for c in constraints
            @test is_state_constraint(c)
        end
        add_constraint!(env,child1,constraints[1])

        @test count_conflicts(child2) == 0
        @test count_conflicts(root_node) == 0

        @test convert_to_vertex_lists(
            get_paths(child1.solution)[1]
            ) == convert_to_vertex_lists(
            get_paths(root_node.solution)[1]
            )

        set_solution_path!(child1.solution,Path([P()]),1)
        @test convert_to_vertex_lists(
            get_paths(child1.solution)[1]
            ) != convert_to_vertex_lists(
            get_paths(root_node.solution)[1]
            )

        detect_conflicts!(child1,[1])
        @test count_conflicts(child1) == 0
    end
end
let
    S = CBSEnv.State
    A = CBSEnv.Action
    P = PathNode{S,A}

    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    env = CBSEnv.LowLevelEnv(graph=G,agent_idx=1)
    isa(state_space_trait(env),DiscreteSpace)
    let
        for v in vertices(G)
            s = S(v,1)
            idx,t = serialize(env,s,s.t)
            sp, _ = deserialize(env,S(),idx,t)
            @test get_vtx(s) == get_vtx(sp)
        end
        for e in edges(G)
            a = A(e,1)
            idx,t = serialize(env,a,1)
            ap, _ = deserialize(env,A(),idx,t)
            @test get_e(a) == get_e(ap)
        end
    end
    let
        for constraints in [
            discrete_constraint_table(env,1),
            # ConstraintTable{P}(agent_id = 1)
            ]
            # add a constraint whose agent id does not match (should throw an error)
            s = S(1,1)
            a = A(Edge(1,2),1)
            sp = S(2,2)
            c = state_constraint(get_agent_id(constraints)+1,P(),2)
            @test_throws AssertionError add_constraint!(env,constraints,c) # wrong agent id
            @test_throws AssertionError CRCBS.has_constraint(env,constraints,c) # wrong agent id
            # add a constraint whose agent id DOES match
            c = state_constraint(get_agent_id(constraints),P(s,a,sp),2)
            add_constraint!(env,constraints,c)
            @test CRCBS.has_constraint(env,constraints,c)
            remove_constraint!(env,constraints,c)
            @test !CRCBS.has_constraint(env,constraints,c)
            add_constraint!(env,constraints,c)
            sc,ac = CRCBS.search_constraints(env,constraints,get_path_node(c))
            @test length(sc) == 1
            @test length(ac) == 0
            # # add a constraint whose agent id DOES match
            c = action_constraint(get_agent_id(constraints),P(s,a,sp),2)
            add_constraint!(env,constraints,c)
            @test CRCBS.has_constraint(env,constraints,c)
            remove_constraint!(env,constraints,c)
            @test !CRCBS.has_constraint(env,constraints,c)
            add_constraint!(env,constraints,c)
            sc,ac = CRCBS.search_constraints(env,constraints,get_path_node(c))
            @test length(sc) == 1
            @test length(ac) == 1
            # # query sorted constraints
            c = sorted_state_constraints(env,constraints)[1]
            @test get_vtx(get_sp(c)) == get_vtx(sp)
            c = sorted_action_constraints(env,constraints)[1]
            @test get_e(get_a(c)) == get_e(a)
            # Test interval constraints
            c = state_constraint(get_agent_id(constraints),P(s,a,sp),3,4)
            add_constraint!(env,constraints,c)
            for t in 3:4
                @test CRCBS.has_constraint(env,constraints,state_constraint(get_agent_id(constraints),P(s,a,sp),t))
            end
            remove_constraint!(env,constraints,c)
            for t in 3:4
                @test !CRCBS.has_constraint(env,constraints,state_constraint(get_agent_id(constraints),P(s,a,sp),t))
            end

        end
    end
end
