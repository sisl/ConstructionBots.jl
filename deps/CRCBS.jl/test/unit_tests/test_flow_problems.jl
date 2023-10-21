let
    G1 = MetaGraph()
    add_vertex!(G1,Dict(:x=>0,:y=>0))
    add_vertex!(G1,Dict(:x=>1,:y=>0))
    add_vertex!(G1,Dict(:x=>2,:y=>0))
    add_vertex!(G1,Dict(:x=>3,:y=>0))
    add_vertex!(G1,Dict(:x=>4,:y=>0))
    # add_vertex!(G1,Dict(:x=>1,:y=>1))
    # add_vertex!(G1,Dict(:x=>3,:y=>1))
    add_edge!(G1,1,2)
    add_edge!(G1,2,3)
    add_edge!(G1,3,4)
    add_edge!(G1,4,5)
    # add_edge!(G1,2,6)
    # add_edge!(G1,4,7)
    start_vtxs = [1, 5]
    goal_vtxs = [2, 3]

    T = 2 # time horizon to extend graph
    # Gx, model, x = construct_ILP_flow_model(G1,T,start_vtxs,goal_vtxs,milp_optimizer())
    # optimize!(model)
    Gx = CRCBS.FlowProblems.extend_G_through_time(G1,T)
    @test nv(Gx) == nv(G1)*(T+1) + ne(G1)*4*T
    x = [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0
        ]
    paths = CRCBS.FlowProblems.extract_solution_from_flow_matrix(Gx,x,start_vtxs,goal_vtxs)
    true_paths = CRCBS.FlowProblems.extract_paths_from_flow_solution(G1,paths)
    for (i,path) in enumerate(true_paths)
        @test path[1] == start_vtxs[i]
        @test path[end] in goal_vtxs
    end
end
