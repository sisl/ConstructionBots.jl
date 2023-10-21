# is_root_node
let
    G = DiGraph(2)
    add_edge!(G,1,2)
    @test is_root_node(G,1)
    @test !is_terminal_node(G,1)
    @test !is_root_node(G,2)
    @test is_terminal_node(G,2)
    @test get_all_root_nodes(G) == Set{Int}(1)
    @test get_all_terminal_nodes(G) == Set{Int}(2)
end
let
    G = Graph(2)
    add_edge!(G,1,2)
    D = get_dist_matrix(G)
    @test D == [0 1; 1 0]
end
let
    G = cycle_digraph(10)
    vtxs = capture_connected_nodes(G,1,v->v<5)
    @test all(v in vtxs for v in 1:4)
    @test !any(v in vtxs for v in 5:10)
    @test isempty(capture_connected_nodes(G,1,v->v>2))

    c = edge_cover(G,[1,2],:all)
    @test Edge(10,1) in c
    @test Edge(1,2) in c
    @test Edge(2,3) in c

    x = exclusive_edge_cover(G,[1,2],:all)
    @test Edge(10,1) in x
    @test !( Edge(1,2) in x )
    @test Edge(2,3) in x
end
