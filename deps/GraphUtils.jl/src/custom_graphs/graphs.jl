export
    AbstractCustomNGraph,
    AbstractCustomNDiGraph,
    AbstractCustomNTree,
    AbstractCustomNETree,
    AbstractCustomTree, 

    get_graph,
    get_vtx_ids,
    get_vtx_map,
    get_nodes,

    get_vtx,
    get_vtx_id,
    get_node,
    get_parent,

    get_edge,
    replace_edge!,
    # delete_edge!,
    # add_custom_edge!,

    node_val,
    edge_val,
    node_id, # may cause an issue
    edge_source,
    edge_target,

    set_vtx_map!,
    insert_to_vtx_map!,

    replace_node!,
    add_node!,
    make_node,
    make_edge,
    add_child!,
    add_parent!,
    rem_node!,
    rem_nodes!

abstract type AbstractCustomGraph <: AbstractGraph{Int} end

"""
    abstract type AbstractCustomNGraph{G,N,ID} <: AbstractCustomGraph

An abstract Custom Graph type, with an underlying graph of type `G`, nodes of
type `N` with ids of type `ID`. All concrete subtypes `CG<:AbstractCustomNGraph`
must implement the following methods: `get_graph(g::CG)`,`get_vtx_ids(g::CG)`,
`get_vtx_map(g::CG)`, and `get_nodes(g::CG)`. These methods are implemented by
default if `g::CG` has the following fields:
- `graph     ::CG`
- `nodes     ::Vector{N}`
- `vtx_map   ::Dict{ID,Int}`
- `vtx_ids   ::Vector{ID}`

Abstract subtypes of `AbstractCustomNGraph{G,N,ID}` include:
- `AbstractCustomNEGraph{G,N,E,ID}` - a graph with custom edges of type `E`
"""
abstract type AbstractCustomNGraph{G,N,ID} <: AbstractCustomGraph end

"""
    abstract type AbstractCustomNEGraph{G,N,E,ID} <: AbstractCustomNGraph{G,N,ID}

An abstract Custom Graph type, with an underlying graph of type `G`, nodes of
type `N` with ids of type `ID`, and edges of type `E`. All concrete subtypes
`CG<:AbstractCustomNEGraph` must implement the required `AbstractCustomNGraph`
interface in addition to the following methods:
- `out_edges(g::CG)` : returns an integer-indexed forward adjacency list `fadj`
    such that `fadj[u::Int][v::Int]` contains the custom edge associated
    with `u → v`.
- `in_edges(g::CG)` : returns an integer-indexed backward adjacency list `badj`
    such that `badj[v::Int][u::Int]` contains the custom edge associated
    with `u → v`.
The above methods are implemented by default if `g::CG` has the following
fields:
- `outedges  ::Vector{Dict{Int,E}}`
- `inedges   ::Vector{Dict{Int,E}}`
"""
abstract type AbstractCustomNEGraph{G,N,E,ID} <: AbstractCustomNGraph{G,N,ID} end

const AbstractCustomNDiGraph{N,ID} = AbstractCustomNGraph{DiGraph,N,ID}
const AbstractCustomNEDiGraph{N,E,ID} = AbstractCustomNEGraph{DiGraph,N,E,ID}

"""
    abstract type AbstractCustomNTree{N,ID} <: AbstractCustomNDiGraph{N,ID}

Abstract custom graph type with tree edge structure.
"""
abstract type AbstractCustomNTree{N,ID} <: AbstractCustomNDiGraph{N,ID} end
abstract type AbstractCustomNETree{N,E,ID} <: AbstractCustomNEDiGraph{N,E,ID} end
const AbstractCustomTree = Union{AbstractCustomNTree,AbstractCustomNETree}

# Common interface
_graph_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID}  = G
_node_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID}   = N
_id_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID}     = ID
"""
    get_graph(g::AbstractCustomNGraph{G,N,ID})

return the underlying graph of type `G`.
"""
get_graph(g::AbstractCustomGraph)   = g.graph
get_graph(g::AbstractGraph)   = g
"""
    get_vtx_ids(g::AbstractCustomNGraph{G,N,ID})

return the vector `Vector{ID}` of `g`'s unique vertex ids.
"""
get_vtx_ids(g::AbstractCustomNGraph) = g.vtx_ids
"""
    get_vtx_map(g::AbstractCustomNGraph{G,N,ID})

return a data structure (e.g, a 'Dict{ID,Int}') mapping node id to node index.
"""
get_vtx_map(g::AbstractCustomNGraph) = g.vtx_map
"""
    get_nodes(g::AbstractCustomNGraph{G,N,ID})

Return the vector `Vector{N}` of `g`'s nodes.
"""
get_nodes(g::AbstractCustomNGraph)   = g.nodes

Base.zero(g::G) where {G<:AbstractCustomNGraph} = G(graph=_graph_type(g)())

get_vtx(g::AbstractCustomNGraph,v::Int) = v
get_vtx(g::AbstractCustomNGraph{G,N,ID},id::ID) where {G,N,ID} = get(get_vtx_map(g), id, -1)
get_vtx(g::AbstractCustomNGraph{G,N,ID},node::N) where {G,N,ID} = get_vtx(g,node.id)
get_vtx_id(g::AbstractCustomNGraph,v::Int)             = get_vtx_ids(g)[v]
get_node(g::AbstractCustomNGraph,v) = get_nodes(g)[get_vtx(g,v)]

for op in [:edgetype,:ne,:nv,:vertices,:edges,:is_cyclic,:topological_sort_by_dfs,:is_directed,:is_connected]
    @eval Graphs.$op(g::AbstractCustomNGraph) = $op(get_graph(g))
end
for op in [:outneighbors,:inneighbors,:indegree,:outdegree,:has_vertex]
    @eval Graphs.$op(g::AbstractCustomNGraph,v::Int) = $op(get_graph(g),v)
    @eval Graphs.$op(g::AbstractCustomNGraph,id) = $op(g,get_vtx(g,id))
end
for op in [:bfs_tree]
    @eval Graphs.$op(g::AbstractCustomNGraph,v::Int;kwargs...) = $op(get_graph(g),v;kwargs...)
    @eval Graphs.$op(g::AbstractCustomNGraph,id;kwargs...) = $op(get_graph(g),get_vtx(g,id);kwargs...)
end
for op in [:has_edge] #,:add_edge!,:rem_edge!]
    @eval Graphs.$op(s::AbstractCustomNGraph,u,v) = $op(get_graph(s),get_vtx(s,u),get_vtx(s,v))
end

"""
    node_id(node)

Return the id of a node. Part of the required node interface for nodes in an
 `AbstractCustomNGraph`.
"""
node_id(node)       = node.id

"""
    node_val(node)

Return the value associated with a node. Part of the optional node interface for
nodes in an `AbstractCustomNGraph`.
"""
node_val(node)      = node.val

"""
    edge_source(edge)

Returns the ID of the source node of an edge. Part of the required interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_source(edge)   = edge.src
"""
    edge_target(edge)

Returns the ID of the target node of an edge. Part of the required interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_target(edge)   = edge.dst
"""
    edge_val(edge)

Returns the value associated with an edge. Part of the optional interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_val(edge)      = edge.val


function set_vtx_map!(g::AbstractCustomNGraph,node,id,v::Int)
    @assert nv(g) >= v
    get_vtx_map(g)[id] = v
    get_nodes(g)[v] = node
end
function insert_to_vtx_map!(g::AbstractCustomNGraph,node,id,idx::Int=nv(g))
    push!(get_vtx_ids(g), id)
    push!(get_nodes(g), node)
    set_vtx_map!(g,node,id,idx)
end

# TODO code a convenient macro to implement this
#
# for op in node_accessor_interface
#     @eval $op(g::AbstractCustomNGraph,v) = $op(get_node(g,v))
#     @eval $op(g::AbstractCustomNGraph) = map(v->$op(get_node(g,v)), vertices(g))
# end
# for op in node_mutator_interface
#     @eval $op(g::AbstractCustomNGraph,v,val) = $op(get_node(g,v),val)
#     @eval $op(g::AbstractCustomNGraph,val) = begin
#         for v in vertices(g)
#             $op(get_node(g,v),val)
#         end
#     end
# end

"""
    replace_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}

Replace the current node associated with `id` with the new node `node`.
"""
function replace_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}
    v = get_vtx(g, id)
    @assert v != -1 "node id $(string(id)) is not in graph and therefore cannot be replaced"
    set_vtx_map!(g,node,id,v)
    node
end

"""
    add_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}

Add `node` to `g` with associated `id`.
"""
function add_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}
    @assert !has_vertex(g, id) "Trying to add node $(string(node)) with id $(string(id)) to g, but a node with id $(string(id)) already exists in g"
    add_vertex!(get_graph(g))
    insert_to_vtx_map!(g,node,id,nv(g))
    add_edge_lists!(g)
    node
end
"""
    make_node(g::AbstractCustomNGraph{G,N,ID},val,id)

Construct a node of type `N` from val and id. This method must be implemented
for whatever custom node type is used.
"""
function make_node end
add_node!(g::AbstractCustomNGraph{G,N,ID},val,id::ID) where {G,N,ID} = add_node!(g,make_node(g,val,id),id)
# add_node!(g::AbstractCustomNEGraph{G,N,E,ID},val,id::ID) where {G,N,E,ID} = add_node!(g,make_node(g,val,id),id)
add_node!(g::AbstractCustomNGraph,val,id) = add_node!(g,make_node(g,val,id))
replace_node!(g::AbstractCustomNGraph{G,N,ID},val,id::ID) where {G,N,ID} = replace_node!(g,make_node(g,val,id),id)

"""
    function add_child!(graph,parent,node,id)

add node `child` to `graph` with id `id`, then add edge `parent` → `child`
"""
function add_child!(g::AbstractCustomNGraph{G,N,ID},parent,child,id) where {G,N,ID}
    n = add_node!(g,child,id)
    if add_edge!(g,parent,id)
        return n
    else
        rem_node!(g,id)
        return nothing
    end
end

"""
    function add_parent!(graph,child,parent,id)

add node `parent` to `graph` with id `id`, then add edge `parent` → `child`
"""
function add_parent!(g::AbstractCustomNGraph{G,N,ID},child,parent::N,id::ID) where {G,N,ID}
    n = add_node!(g,parent,id)
    if add_edge!(g,id,child)
        return n
    else
        rem_node!(g,id)
        return nothing
    end
end
for op in [:add_child!,:add_parent!]
    @eval $op(g::AbstractCustomNGraph{G,N,ID},u,v::N) where {G,N,ID} = $op(g,u,v,v.id)
end

"""
    swap_with_end_and_delete!(vec,v)

Replaces `vec[v]` with `last(vec)`, then removes the last element from `vec`
"""
function swap_with_end_and_delete!(vec::Vector,v)
    vec[v] = last(vec)
    pop!(vec)
    return vec
end

"""
    rem_node!

removes a node (by id) from g.
Note about LightGraphs.rem_vertex!:
"internally the removal is performed swapping the vertices `v` and `nv(G)`, and 
removing the last vertex `nv(G)` from the graph"
"""
function rem_node!(g::AbstractCustomNGraph{G,N,ID}, id::ID) where {G,N,ID}
    v = get_vtx(g, id)
    rem_vertex!(get_graph(g), v)
    swap_with_end_and_delete!(get_nodes(g),v)
    swap_with_end_and_delete!(get_vtx_ids(g),v)
    # deleteat!(get_nodes(g), v)
    delete!(get_vtx_map(g), id)
    # if v was not at end, need to update id_map too.
    if v <= nv(g)
        get_vtx_map(g)[get_vtx_ids(g)[v]] = v
    end
    # deleteat!(get_vtx_ids(g), v)
    # @assert length(get_vtx_ids(g)) == length(get_vtx_map(g)) == nv(get_graph(g))
    # for vtx in v:nv(get_graph(g))
    #     n_id = get_vtx_ids(g)[vtx]
    #     get_vtx_map(g)[n_id] = vtx
    # end
    delete_from_edge_lists!(g,v) # no effect except for AbstractCustomNEGraph
    g
end
rem_node!(g::AbstractCustomNGraph, v) = rem_node!(g,get_vtx_id(g,v))
function rem_nodes!(g::AbstractCustomNGraph, vtxs::Vector)
    node_ids = collect(map(v->get_vtx_id(g,v), vtxs))
    for id in node_ids
        rem_node!(g,id)
    end
    g
end

# Edge graph interface
_edge_type(::AbstractCustomNEGraph{G,N,E,ID}) where {G,N,E,ID} = E

"""
    in_edges(g::AbstractCustomNEGraph{G,N,E,ID})

Returns an integer-indexed backward adjacency list `badj` (e.g.,
`badj::Vector{Dict{Int,E}}`) such that `badj[v::Int][u::Int]` contains the
custom edge associated with `u → v`.
"""
in_edges(g::AbstractCustomNEGraph) = g.inedges
in_edges(g::AbstractCustomNEGraph, v) = in_edges(g)[get_vtx(g,v)]
in_edge(g::AbstractCustomNEGraph, v, u) = in_edges(g,v)[get_vtx(g,u)]
function set_edge!(g::AbstractCustomNEGraph{G,N,E,ID}, u,v, edge::E) where {G,N,E,ID}
    out_edges(g,u)[get_vtx(g,v)] = edge
    in_edges(g,v)[get_vtx(g,u)] = edge
    # TODO for undirected graph, insert reversed edge too?
    edge
end

"""
    out_edges(g::AbstractCustomNEGraph{G,N,E,ID})

Returns an integer-indexed forward adjacency list `fadj` (e.g.,
`fadj::Vector{Dict{Int,E}}`) such that `fadj[u::Int][v::Int]` contains the
custom edge associated with `u → v`.
"""
out_edges(g::AbstractCustomNEGraph) = g.outedges
out_edges(g::AbstractCustomNEGraph, v) = out_edges(g)[get_vtx(g,v)]
out_edge(g::AbstractCustomNEGraph, u, v) = out_edges(g,u)[get_vtx(g,v)]
function get_edge(g::AbstractCustomNEGraph,u,v)
    @assert has_edge(g,u,v) "Edge $u → $v does not exist."
    return out_edge(g,u,v)
end
get_edge(g::AbstractCustomNEGraph,edge) = get_edge(g,edge_source(edge),edge_target(edge))
Graphs.has_edge(g::AbstractCustomNEGraph,e) = has_edge(g,edge_source(e),edge_target(e))

add_edge_lists!(g::AbstractCustomNGraph) = g
function add_edge_lists!(g::AbstractCustomNEGraph{G,N,E,ID}) where {G,N,E,ID}
    push!(g.outedges, Dict{Int,E}())
    push!(g.inedges, Dict{Int,E}())
    return g
end
delete_from_edge_lists!(g::AbstractCustomNGraph,v::Int) = g
function delete_from_edge_lists!(g::AbstractCustomNEGraph,v::Int)
    swap_with_end_and_delete!(in_edges(g),v)
    swap_with_end_and_delete!(out_edges(g),v)
    # deleteat!(in_edges(g),v)
    # deleteat!(out_edges(g),v)
    return g
end

# TODO Make a GraphType with default constructible edges
function add_custom_edge!(g::AbstractCustomNEGraph{G,N,E,ID},u,v,edge::E) where {G,N,E,ID}
    if !is_legal_edge(g,u,v,edge)
        @warn "An edge $u → $v is illegal in $g"
        return false
    end
    if add_edge!(get_graph(g),get_vtx(g,u),get_vtx(g,v))
        set_edge!(g,u,v,edge)
        return true
    end
    @warn "Cannot add edge $u → $v. Does an edge already exist?"
    # println("Cannot add edge $u → $v. Does an edge already exist?")
    return false
end
add_custom_edge!(g::AbstractCustomNEGraph,edge) = add_custom_edge!(g,edge_source(edge),edge_target(edge),edge)
# LightGraphs.add_edge!(g::AbstractCustomNEGraph,args...) = add_custom_edge!(g,args...)

add_custom_edge!(g::AbstractCustomNGraph,edge) = add_custom_edge!(g,edge_source(edge),edge_target(edge))
add_custom_edge!(g::AbstractCustomNGraph,u,v) = add_edge!(get_graph(g),get_vtx(g,u),get_vtx(g,v))
add_custom_edge!(g::AbstractCustomNGraph,u,v,args...) = add_custom_edge!(g,u,v)
# LightGraphs.add_edge!(g::AbstractCustomNGraph,u,v,edge) = add_custom_edge!(g,u,v) # no custom edge type here
# LightGraphs.add_edge!(g::AbstractCustomNGraph,u,v) = add_custom_edge!(g,u,v) # no custom edge type here
Graphs.add_edge!(g::AbstractCustomGraph,args...) = add_custom_edge!(g,args...) # no custom edge type here

"""
    make_edge(g::G,u,v,val) where {G}

Construct an edge `u → v` of type `_edge_type(g)` based on val. Default behavior
is to throw an error.
"""
function make_edge(g::G,u,v,val) where {G}
    throw(ErrorException(
    """
    MethodError: `make_edge(g,u,v,val)` not implemented.
    To add an edge to a graph g::$G, either:
    - add the edge explicitly using `add_edge!(g,edge,[u,v])
    - make a default constructor for the edge type
    - implement `GraphUtils.make_edge(g,u,v,val)`
    """
    ))
end
make_edge(g,u,v) = make_edge(g,u,v,nothing) 
function add_custom_edge!(g::AbstractCustomNEGraph,u,v,val)
    add_custom_edge!(g,u,v,make_edge(g,u,v,val))
end
function add_custom_edge!(g::AbstractCustomNEGraph,u,v)
    add_custom_edge!(g,u,v,make_edge(g,u,v))
end
# function LightGraphs.add_edge!(g::AbstractCustomNEGraph,u,v,val)
#     add_edge!(g,u,v,make_edge(g,u,v,val))
# end
# function LightGraphs.add_edge!(g::AbstractCustomNEGraph,u,v)
#     add_edge!(g,u,v,make_edge(g,u,v))
# end
function replace_edge!(g::AbstractCustomNEGraph{G,N,E,ID},u,v,edge::E) where {G,N,E,ID}
    if has_edge(g,u,v)
        set_edge!(g,u,v,edge)
        return true
    end
    @warn "graph does not have edge $u → $v"
    return false
end
replace_edge!(g::AbstractCustomNEGraph,u,v,val) = replace_edge!(g,u,v,make_edge(g,u,v,val))
function replace_edge!(g::AbstractCustomNEGraph{G,N,E,ID},old_edge,edge::E) where {G,N,E,ID}
    replace_edge!(g,edge_source(old_edge),edge_target(old_edge),edge)
end

function delete_edge!(g::AbstractCustomNEGraph, u, v)
    if rem_edge!(get_graph(g),get_vtx(g,u),get_vtx(g,v))
        delete!(out_edges(g,u),get_vtx(g,v))
        delete!(in_edges(g,v),get_vtx(g,u))
        return true
    end
    @warn "Cannot remove edge $u → $v. Does it exist?"
    return false
end
delete_edge!(g::AbstractCustomNGraph, u, v) = rem_edge!(get_graph(g),get_vtx(g,u),get_vtx(g,v))
delete_edge!(g,edge) = delete_edge!(g,edge_source(edge),edge_target(edge))
Graphs.rem_edge!(g::AbstractCustomGraph, args...) = delete_edge!(g,args...)
# LightGraphs.rem_edge!(g::AbstractCustomNEGraph, u, v) = delete_edge!(g,u,v)
# LightGraphs.rem_edge!(g::AbstractCustomNEGraph, edge) = rem_edge!(g,edge_source(edge),edge_target(edge))

# Tree interface
get_parent(g::AbstractCustomTree,v) = get(inneighbors(g,v),1,-1)
is_legal_edge(g,u,v) = true
is_legal_edge(g,u,v,e) = is_legal_edge(g,u,v)
is_legal_edge(g::AbstractCustomTree,u,v) = !(has_vertex(g,get_parent(g,v)) || get_vtx(g,u) == get_vtx(g,v))
# function LightGraphs.add_edge!(g::AbstractCustomNTree,u,v)
function add_custom_edge!(g::AbstractCustomTree,u,v)
    is_legal_edge(g,u,v)
    if !is_legal_edge(g,u,v)
        return false
    end
    add_edge!(get_graph(g),get_vtx(g,u),get_vtx(g,v))
end

################################################################################
################################ Concrete Types ################################
################################################################################

export
    CustomNode,
    CustomEdge,
    CustomNGraph,
    CustomNDiGraph,
    CustomNEGraph,
    CustomNEDiGraph,
    CustomNTree,
    CustomNETree,
    NGraph,
    NEGraph,
    NTree,
    NETree

"""
    CustomGraph

An example concrete subtype of `AbstractCustomNGraph`.
"""
@with_kw struct CustomNGraph{G<:AbstractGraph,N,ID} <: AbstractCustomNGraph{G,N,ID}
    graph               ::G                     = G()
    nodes               ::Vector{N}             = Vector{N}()
    vtx_map             ::Dict{ID,Int}          = Dict{ID,Int}()
    vtx_ids             ::Vector{ID}            = Vector{ID}() # maps vertex uid to actual graph node
end
const CustomNDiGraph{N,ID} = CustomNGraph{DiGraph,N,ID}

"""
    CustomNEGraph{G,N,E,ID}

Custom graph type with custom edge and node types.
"""
@with_kw struct CustomNEGraph{G,N,E,ID} <: AbstractCustomNEGraph{G,N,E,ID}
    graph               ::G                     = G()
    nodes               ::Vector{N}             = Vector{N}()
    vtx_map             ::Dict{ID,Int}          = Dict{ID,Int}()
    vtx_ids             ::Vector{ID}            = Vector{ID}()
    inedges             ::Vector{Dict{Int,E}}   = Vector{Dict{Int,E}}()
    outedges            ::Vector{Dict{Int,E}}   = Vector{Dict{Int,E}}()
end
const CustomNEDiGraph{N,E,ID} = CustomNEGraph{N,E,ID}

"""
    CustomNTree

An example concrete subtype of `AbstractCustomNTree`.
"""
@with_kw struct CustomNTree{N,ID} <: AbstractCustomNTree{N,ID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{N}             = Vector{N}()
    vtx_map             ::Dict{ID,Int}           = Dict{ID,Int}()
    vtx_ids             ::Vector{ID}             = Vector{ID}()
end

"""
    CustomNETree{G,N,E,ID}

Custom tree type with custom edge and node types.
"""
@with_kw struct CustomNETree{N,E,ID} <: AbstractCustomNETree{N,E,ID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{N}             = Vector{N}()
    vtx_map             ::Dict{ID,Int}          = Dict{ID,Int}()
    vtx_ids             ::Vector{ID}            = Vector{ID}()
    inedges             ::Vector{Dict{Int,E}}   = Vector{Dict{Int,E}}()
    outedges            ::Vector{Dict{Int,E}}   = Vector{Dict{Int,E}}()
end

"""
    CustomNode{N,ID}

A custom node type. Fields:
- `id::ID`
- `val::N`
"""
struct CustomNode{N,ID}
    id::ID
    val::N
end
CustomNode{N,ID}(id::ID) where {N,ID} = CustomNode{N,ID}(id,N())
function make_node(g::AbstractCustomNGraph{G,CustomNode{N,ID},ID},val::N,id) where {G,N,ID}
    _node_type(g)(id,val)
end
# function make_node(g::AbstractCustomNEGraph{G,CustomNode{N,ID},E,ID},val::N,id) where {G,N,E,ID}
#     _node_type(g)(id,val)
# end
"""
    CustomEdge{E,ID}

A custom node type. Fields:
- `id::ID`
- `val::E`
"""
struct CustomEdge{E,ID}
    src::ID
    dst::ID
    val::E
end
CustomEdge{E,ID}(id1::ID,id2::ID) where {E,ID} = CustomEdge{E,ID}(id1,id2,E())
function make_edge(g::AbstractCustomNEGraph{G,N,CustomEdge{E,ID},ID},u,v,val::E) where {G,N,E,ID}
    _edge_type(g)(
        get_vtx_id(g,get_vtx(g,u)),
        get_vtx_id(g,get_vtx(g,v)),
        val)
end

# """
#     abstract type CustomNGraph{G,N,ID} <: AbstractCustomNGraph{G,CustomNode{N,ID},ID}
#
# Abstract custom graph with nodes of type `CustomNode{N,ID}`.
# """
# abstract type CustomNGraph{G,N,ID} <: AbstractCustomNGraph{G,CustomNode{N,ID},ID} end
# """
#     abstract type CustomNEGraph{G,N,ID} <: AbstractCustomNEGraph{G,CustomEdge{E,ID},CustomNode{N,ID},ID}
#
# Abstract custom graph with nodes of type `CustomNode{N,ID}` and edges of type
# `CustomEdge{E,ID}`.
# """
# abstract type CustomNEGraph{G,N,E,ID} <: AbstractCustomNEGraph{G,CustomEdge{E,ID},CustomNode{N,ID},ID} end
const NGraph{G,N,ID} = CustomNGraph{G,CustomNode{N,ID},ID}
const NEGraph{G,N,E,ID} = CustomNEGraph{G,CustomNode{N,ID},CustomEdge{E,ID},ID}
const NTree{N,ID} = CustomNTree{CustomNode{N,ID},ID}
const NETree{N,E,ID} = CustomNETree{CustomNode{N,ID},CustomEdge{E,ID},ID}

for T in (:CustomNGraph,:CustomNTree)
    @eval begin
        function Base.convert(::Type{X},g::AbstractCustomNGraph{G,N,ID}) where {G,N,ID,X<:$T}
            X(
                deepcopy(get_graph(g)),
                deepcopy(get_nodes(g)),
                deepcopy(get_vtx_map(g)),
                deepcopy(get_vtx_ids(g))
            )
        end
    end
end
for T in (:CustomNEGraph,:CustomNETree)
    @eval begin
        function Base.convert(::Type{X},g::AbstractCustomNEGraph{G,N,E,ID}) where {G,N,E,ID,X<:$T}
            X(
                deepcopy(get_graph(g)),
                deepcopy(get_nodes(g)),
                deepcopy(get_vtx_map(g)),
                deepcopy(get_vtx_ids(g)),
                deepcopy(in_edges(g)),
                deepcopy(out_edges(g)),
            )
        end
    end
end
