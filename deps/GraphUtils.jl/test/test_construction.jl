let
    K = initialize_regular_vtx_grid()
    G = initialize_grid_graph_from_vtx_grid(K)
    @test nv(G) == sum(K .!= 0)

    x_dim = size(K,1)
    y_dim = size(K,2)
    vtxs = [(i,j) for i in 1:x_dim for j in 1:y_dim if K[i,j] != 0]
    K2 = construct_vtx_grid(vtxs,(x_dim,y_dim))
    G2 = initialize_grid_graph_from_vtx_grid(K2)

    @test nv(G) == nv(G2)
    @test ne(G) == ne(G2)
end
let
    G = initialize_regular_grid_graph()
    initialize_grid_graph_with_obstacles((10,10),[(1,2),(3,4)])
    construct_vtx_grid([(1,2),(3,4)],(10,10))
end
