# module Connectivity

# using LightGraphs

export
    is_root_node,
    is_terminal_node,
    get_all_root_nodes,
    get_all_terminal_nodes,
    get_dist_matrix


is_root_node(G,v) = indegree(G,v) == 0 # length(inneighbors(G,v)) == 0

"""
    isroot_node(G,v)

Inputs:
    `G` - graph
    `v` - query vertex

Outputs:
    returns `true` if vertex v has no outneighbors
"""
is_terminal_node(G,v) = outdegree(G,v) == 0 # length(outneighbors(G,v)) == 0

"""
    get_all_root_nodes
"""
function get_all_root_nodes(G)
    root_nodes = Set{Int}()
    for v in vertices(G)
        if is_root_node(G,v)
            push!(root_nodes,v)
        end
    end
    return root_nodes
end

"""
    get_all_terminal_nodes
"""
function get_all_terminal_nodes(G)
    root_nodes = Set{Int}()
    for v in vertices(G)
        if is_terminal_node(G,v)
            push!(root_nodes,v)
        end
    end
    return root_nodes
end

"""
    get_dist_matrix(G)

Get the distance matrix corresponding to the edge weights of a graph
"""
function get_dist_matrix(G)
    distmx = zeros(Float64,nv(G),nv(G))
    for v in vertices(G)
        distmx[v,:] .= dijkstra_shortest_paths(G,v).dists
    end
    distmx
end
function get_dist_matrix(graph::G,weight_mtx::M) where {G,M}
    D = zeros(Int,nv(graph),nv(graph))
    for v1 in vertices(graph)
        ds = dijkstra_shortest_paths(graph,v1,weight_mtx)
        D[v1,:] = ds.dists
    end
    D
end

export edge_cover

"""
    edge_cover(G,vtxs,mode=:all)

Get all edges coming from vertices `vtxs`. If `mode=:in`, just collect incoming
edges. If `:out`, just outgoing. If `:all`, both.
"""
function edge_cover(G,vtxs,mode=:all)
    cover = Set{Edge}()
    for v in vtxs
        if mode == :all || mode == :in
            for v2 in inneighbors(G,v)
                push!(cover,Edge(v2,v))
            end
        end
        if mode == :all || mode == :out
            for v2 in outneighbors(G,v)
                push!(cover,Edge(v,v2))
            end
        end
    end
    return cover
end

export exclusive_edge_cover
"""
    exclusive_edge_cover(G,vtxs,mode)

Returns the edge cover, minus all edges whose sources and edges are both
contained within `vtxs`.
"""
function exclusive_edge_cover(G,vtxs,mode=:all)
    Set{Edge}(e for e in edge_cover(G,vtxs,mode) if !(e.dst in vtxs && e.src in vtxs))
end

export capture_connected_nodes

"""
    capture_connected_nodes(G,vtxs,f)

Collect the connected set of nodes in `G` that intersects with `vtxs` and such
that `f(v) == true` for each `v` in the set.
Args:
* `vtxs::Union{Int,Vector{Int},Set{Int}}`
"""
function capture_connected_nodes(G,vtxs,f)
    frontier = Set{Int}(vtxs)
    explored = Set{Int}()
    captured = Set{Int}()
    while !isempty(frontier)
        v = pop!(frontier)
        if f(v)
            push!(captured,v)
            for vp in inneighbors(G,v)
                if !(vp in explored)
                    push!(frontier,vp)
                end
            end
            for vp in outneighbors(G,v)
                if !(vp in explored)
                    push!(frontier,vp)
                end
            end
        end
        push!(explored,v)
    end
    return captured
end

# end
