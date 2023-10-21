# `topological_sort`
let
    # simple graph
    G = DiGraph(4)
    add_edge!(G,1,2)
    add_edge!(G,1,4)
    add_edge!(G,2,3)
    add_edge!(G,3,4)
    ordering = GraphUtils.topological_sort(G)
    @test ordering == [1,2,3,4]  
end
let
    # disjoint graph
    G = DiGraph(4)
    add_edge!(G,1,2)
    add_edge!(G,3,4)
    ordering = GraphUtils.topological_sort(G)
    @test length(ordering) == nv(G)
    @test findfirst(ordering .== 1) < findfirst(ordering .== 2)
    @test findfirst(ordering .== 3) < findfirst(ordering .== 4)
end
let
    # cyclic graph (should throw error)
    G = DiGraph(3)
    add_edge!(G,1,2)
    add_edge!(G,2,3)
    add_edge!(G,3,1)
    
    @test_throws AssertionError GraphUtils.topological_sort(G)
end
# `find_index_in_sorted_array` and `insert_to_sorted_array`
let
    array = [1.0,2.0,3.0,4.0,5.0]
    array = insert_to_sorted_array!(array, 3.5)
    @test(array[4] == 3.5)
    array = insert_to_sorted_array!(array, 0.5)
    @test(array[1] == 0.5)
    array = insert_to_sorted_array!(array, 5.5)
    @test(array[end] == 5.5)
end
let
    v = [0,1,2,4,5,6]
    L = length(v)
    x = 3
    @test find_index_in_sorted_array(v,x) == 4
    insert_to_sorted_array!(v,x)
    @test length(v) == L + 1
    v[4] == x
end
let
    v = [0,1,1,1,1,2]
    L = length(v)
    x = 1
    @test find_index_in_sorted_array(v,x) == 2
end
let
    v = Vector{Int}()
    insert_to_sorted_array!(v,1)
    @test length(v) == 1
end
