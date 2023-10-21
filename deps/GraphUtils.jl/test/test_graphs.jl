# test backup_ancestors and backup_descendants
let
    NODE_TYPE=Int
    ID_TYPE=Symbol
    my_tree = GraphUtils.NTree{NODE_TYPE,ID_TYPE}()
    add_node!(my_tree,1,:ONE)
    add_node!(my_tree,2,:TWO)
    add_node!(my_tree,3,:THREE)
    @test add_edge!(my_tree,:ONE,:TWO)
    @test add_edge!(my_tree,:TWO,:THREE)

    d_map = backup_descendants(my_tree,n->node_id(n)==:TWO)
    @test d_map[:THREE] === nothing
    @test d_map[:TWO] == :TWO
    @test d_map[:ONE] == :TWO
    d_map = GraphUtils.backup_multiple_descendants(my_tree,n->node_id(n)==:TWO)
    @test isempty(d_map[:THREE])
    @test :TWO in d_map[:TWO]
    @test :TWO in d_map[:ONE]
    d_map = backup_ancestors(my_tree,n->node_id(n)==:TWO)
    @test d_map[:THREE] == :TWO
    @test d_map[:TWO] == :TWO
    @test d_map[:ONE] === nothing
    d_map = GraphUtils.backup_multiple_ancestors(my_tree,n->node_id(n)==:TWO)
    @test :TWO in d_map[:THREE]
    @test :TWO in d_map[:TWO]
    @test isempty(d_map[:ONE])

end
let
    NODE_TYPE=Int
    ID_TYPE=Symbol
    my_tree = GraphUtils.NTree{NODE_TYPE,ID_TYPE}()
    add_node!(my_tree,1,:ONE)
    add_node!(my_tree,2,:TWO)
    add_node!(my_tree,3,:THREE)
    @test add_edge!(my_tree,:ONE,:TWO)
    # Vertex :TWO already has a parent--can't have another.
    @test !add_edge!(my_tree,:THREE,:TWO)
    add_child!(my_tree,:THREE,4,:FOUR)
    @test has_edge(my_tree,:THREE,:FOUR)

    d_map = backup_descendants(my_tree,n->node_id(n)==:FOUR)
    @test d_map[:FOUR] == :FOUR
    @test d_map[:THREE] == :FOUR
    @test d_map[:TWO] === nothing
    @test d_map[:ONE] === nothing

    d_map = backup_descendants(my_tree,n->node_id(n)==:THREE)
    @test d_map[:FOUR] === nothing
    @test d_map[:THREE] == :THREE
    @test d_map[:TWO] === nothing
    @test d_map[:ONE] === nothing

    d_map = backup_ancestors(my_tree,n->node_id(n)==:THREE)
    @test d_map[:FOUR] === :THREE
    @test d_map[:THREE] == :THREE
    @test d_map[:TWO] === nothing
    @test d_map[:ONE] === nothing
end
let
    NODE_TYPE = Int
    ID_TYPE = Symbol
    EDGE_TYPE = GraphUtils.CustomEdge{Int,ID_TYPE}
    g = GraphUtils.CustomNEGraph{DiGraph,NODE_TYPE,EDGE_TYPE,ID_TYPE}()
    e = GraphUtils.CustomEdge(:ONE,:TWO,1+2)
    e2 = GraphUtils.CustomEdge(:TWO,:THREE,2+3)
    # EDGE_TYPE = Int
    # g = GraphUtils.NEGraph{DiGraph,NODE_TYPE,EDGE_TYPE,ID_TYPE}()
    # e = 3
    # e2 = 5
    isa(g,GraphUtils.AbstractCustomNGraph)
    add_node!(g,1,:ONE)
    add_node!(g,2,:TWO)
    add_node!(g,3,:THREE)
    @test add_edge!(g,:ONE,:TWO,e)
    @test !has_edge(g,:TWO,:ONE)
    @test add_edge!(g,:TWO,:THREE,e2)
    rem_node!(g,:THREE)
    @test !has_vertex(g,:THREE)
    @test !has_edge(g,:TWO,:THREE)
    @test !has_edge(g,e2)

    make_edge(g,:TWO,:ONE,-1)
    GraphUtils.make_edge(g,u::Symbol,v::Symbol) = GraphUtils.make_edge(g,u,v,-1)
    @test add_edge!(g,:TWO,:ONE)
    @test edge_val(get_edge(g,:TWO,:ONE)) == -1
    rem_edge!(g,:TWO,:ONE)
    @test !has_edge(g,:TWO,:ONE)
    @test add_edge!(g,:TWO,:ONE,-2)
    @test edge_val(get_edge(g,:TWO,:ONE)) == -2
end
let
    G = DiGraph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,3)
    @test depth_first_search(G,1,
        v->v==3,
        v->true,
        outneighbors
    ) == 3
    @test depth_first_search(G,1,
        v->v==3,
        v->false,
        outneighbors
    ) == -1
    @test depth_first_search(G,3,
        v->v==1,
        v->true,
        inneighbors
    ) == 1
    @test depth_first_search(G,1,
        v->v>=1,
        v->true,
        outneighbors
    ) == 1
    @test depth_first_search(G,1,
        v->v>=1,
        v->true,
        outneighbors;
        skip_first=true
    ) == 2

    @test collect_ancestors(G,2) == Set(1)
    @test collect_ancestors(G,3) == Set([1,2])
    @test collect_descendants(G,2) == Set(3)
    @test collect_descendants(G,1) == Set([2,3])
    @test collect_descendants(G,[1]) == Set([2,3])

    keep = true
    @test collect_descendants(G,[1],keep) == Set([1,2,3])

end
let
    graph = DiGraph(7)
    add_edge!(graph,1,3)
    add_edge!(graph,2,3)
    add_edge!(graph,4,6)
    add_edge!(graph,5,6)
    add_edge!(graph,3,7)
    add_edge!(graph,6,7)
    for (g,target) in [
        (graph,[1,1,2,1,1,2,3]),
        (reverse(graph),[3,3,2,3,3,2,1])
    ]
        for iter in [
                GraphUtils.BFSIterator(g),
                GraphUtils.SortedBFSIterator(g),
                ]
            depths = ones(Int,nv(g))
            while !isempty(iter)
                v = pop!(iter)
                if indegree(g,v) > 0
                    depths[v] = max(depths[v],1+maximum(depths[vp] for vp in inneighbors(g,v)))
                end 
                GraphUtils.update_iterator!(iter,v)
            end
            @test all(depths .== target)
        end
    end
    @test length(collect(BFSIterator(graph))) == nv(graph)
    @test collect(SortedBFSIterator(graph)) == [1, 2, 4, 5, 3, 6, 7]

end

let 
    G = NGraph{DiGraph,String,String}()
    for s in ["A","B","C","D","E"]
        add_node!(G,s,s)
    end
    edge_set = Set([("A","B"),("B","C"),("C","D"),("B","E")])
    for e in edge_set
        add_edge!(G,e[1],e[2])
    end
    for v in [3,2,1,1,1]
        rem_node!(G,v)
        for e in edges(G)
            @test (
                node_val(get_node(G,e.src)),
                node_val(get_node(G,e.dst))) in edge_set
        end
    end

end