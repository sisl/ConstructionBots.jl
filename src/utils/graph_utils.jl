
VALID_ID_COUNTERS = Dict{DataType,Int}()
INVALID_ID_COUNTERS = Dict{DataType,Int}()

abstract type AbstractRobotType end

struct DeliveryBot <: AbstractRobotType end

const DefaultRobotType = DeliveryBot

abstract type AbstractID end

"""
    struct TemplatedID{T} <: AbstractID

A simple way to dispatch by Node type.
"""
struct TemplatedID{T} <: AbstractID
    id::Int
end

@with_kw struct ObjectID <: AbstractID
    id::Int = -1
end

@with_kw struct BotID{R<:AbstractRobotType} <: AbstractID
    id::Int = -1
end

const RobotID = BotID{DeliveryBot}

@with_kw struct LocationID <: AbstractID
    id::Int = -1
end

@with_kw struct ActionID <: AbstractID
    id::Int = -1
end

@with_kw struct OperationID <: AbstractID
    id::Int = -1
end

"""
	AgentID
Special helper for identifying agents.
"""
@with_kw struct AgentID <: AbstractID
    id::Int = -1
end

"""
	VtxID
Special helper for identifying schedule vertices.
"""
@with_kw struct VtxID <: AbstractID
    id::Int = -1
end

Base.summary(id::A) where {A<:AbstractID} = string(string(A), "(", get_id(id), ")")
Base.string(id::A) where {A<:AbstractID} = summary(id)

mutable struct Toggle
    status::Bool
end

function set_toggle_status!(t::Toggle, val = true)
    t.status = val
end

get_toggle_status(t::Toggle) = copy(t.status)

mutable struct Counter
    status::Int
end

function set_counter_status!(t::Counter, val::Int)
    t.status = val
end

get_counter_status(t::Counter) = copy(t.status)

"""
    CachedElement{E}

A mutable container for caching things.
"""
mutable struct CachedElement{E}
    element::E
    is_up_to_date::Bool
    timestamp::Float64
end

CachedElement(element, flag) = CachedElement(element, flag, time())
CachedElement(element) = CachedElement(element, false)

"""
    transform_iter(f, it) = Base.Iterators.accumulate((a,b)->f(b), it)

Transform iterator that applies `f` to each element of `it`.
"""
transform_iter(f, it) = (f(v) for v in it)

"""
    is_root_node(G, v)

Inputs:
    `G` - graph
    `v` - query vertex

Outputs:
    returns `true` if vertex v has no inneighbors
"""
is_root_node(G, v) = indegree(G, v) == 0

"""
is_terminal_node(G, v)

Inputs:
    `G` - graph
    `v` - query vertex

Outputs:
    returns `true` if vertex v has no inneighbors
"""
is_terminal_node(G, v) = outdegree(G, v) == 0

"""
    get_all_root_nodes
"""
function get_all_root_nodes(G)
    root_nodes = Set{Int}()
    for v in Graphs.vertices(G)
        if is_root_node(G, v)
            push!(root_nodes, v)
        end
    end
    return root_nodes
end

"""
    get_all_terminal_nodes
"""
function get_all_terminal_nodes(G)
    root_nodes = Set{Int}()
    for v in Graphs.vertices(G)
        if is_terminal_node(G, v)
            push!(root_nodes, v)
        end
    end
    return root_nodes
end

"""
    get_element(n::CachedElement)

Retrieve the element stored in n.
"""
get_element(n::CachedElement) = n.element

"""
    is_up_to_date(n::CachedElement)

Check if the n is up to date.
"""
is_up_to_date(n::CachedElement) = n.is_up_to_date

"""
    time_stamp(n::CachedElement)

Return the time_stamp at which n was last modified.
"""
time_stamp(n::CachedElement) = n.timestamp

"""
    set_up_to_date!(n::CachedElement,val::Bool=true)

Set the time stamp of n.
"""
function set_up_to_date!(n::CachedElement, val::Bool = true)

    n.is_up_to_date = val
end

"""
    set_element!(n::CachedElement,g)

Set the element stored in n. Does NOT set the `is_up_to_date` flag.
"""
function set_element!(n::CachedElement, g)

    n.element = g
end

"""
    set_time_stamp!(n::CachedElement,t=time())

Set the time stamp of n.
"""
function set_time_stamp!(n::CachedElement, t = time())

    n.timestamp = t
end

"""
    update_element!(n::CachedElement,g)

Set the element of n to g, update the `is_up_to_date` flag and the time stamp.
"""
function update_element!(n::CachedElement, g)

    set_element!(n, g)
    set_up_to_date!(n, true)
    set_time_stamp!(n)
    return g
end

"""
    Base.copy(e::CachedElement)

Shares the e.element, since it doesn't need to be replaced until `set_element!`
is called. Copies `e.is_up_to_date` to preserve the cache state.
"""
Base.copy(e::CachedElement) = CachedElement(e.element, copy(is_up_to_date(e)), time())

Base.convert(::Type{CachedElement{T}}, e::CachedElement) where {T} =
    CachedElement{T}(get_element(e), is_up_to_date(e), time())

const cached_element_accessor_interface = [:is_up_to_date, :time_stamp]

const cached_element_mutator_interface =
    [:update_element!, :set_time_stamp!, :set_element!, :set_up_to_date!]

cross_product_operator(x) = SMatrix{3,3}([
    0.0 -x[3] x[2]
    x[3] 0.0 -x[1]
    -x[2] x[1] 0.0
])

wrap_to_2pi(θ) = mod(θ, 2π)

"""
    wrap_to_pi(θ₀)

wraps the angle θ₀ to a value in (-π,π]
"""
function wrap_to_pi(θ₀)
    θ = wrap_to_2pi(θ₀)
    if θ > π
        θ = θ - 2π
    elseif θ <= -π
        θ += 2π
    end
    return θ
end

"""
    wrap_idx(n,idx)

Wrap index to a one-dimensional array of length `n`
"""
wrap_idx(n, idx) = mod(idx - 1, n) + 1

"""
    wrap_get

Index into array `a` by first wrapping the indices `idx`.
"""
function wrap_get(a::A, idxs) where {R,N,A<:AbstractArray{R,N}}
    a[map(i -> wrap_idx(size(a, i), idxs[i]), 1:N)...]
end

function get_unique_id(::Type{T}) where {T}
    global VALID_ID_COUNTERS
    id = get!(VALID_ID_COUNTERS, T, 1)
    VALID_ID_COUNTERS[T] += 1
    return T(id)
end

function reset_id_counter!(::Type{T}) where {T}
    global VALID_ID_COUNTERS
    VALID_ID_COUNTERS[T] = 1
end

function reset_all_id_counters!()
    global VALID_ID_COUNTERS
    for k in collect(keys(VALID_ID_COUNTERS))
        reset_id_counter!(k)
    end
end

function get_unique_invalid_id(::Type{T}) where {T}
    global INVALID_ID_COUNTERS
    id = get!(INVALID_ID_COUNTERS, T, -1)
    INVALID_ID_COUNTERS[T] -= 1
    return T(id)
end

function reset_invalid_id_counter!(::Type{T}) where {T}
    global INVALID_ID_COUNTERS
    INVALID_ID_COUNTERS[T] = -1
end

function reset_all_invalid_id_counters!()
    global INVALID_ID_COUNTERS
    for k in collect(keys(INVALID_ID_COUNTERS))
        reset_invalid_id_counter!(k)
    end
end

get_id(id::AbstractID) = id.id
Base.:+(id::A, i::Int) where {A<:AbstractID} = A(get_id(id) + i)
Base.:+(id::A, i::A) where {A<:AbstractID} = A(get_id(id) + get_id(i))
Base.:-(id::A, i::Int) where {A<:AbstractID} = A(get_id(id) - i)
Base.:-(id::A, i::A) where {A<:AbstractID} = A(get_id(id) - get_id(i))
Base.:(<)(id1::AbstractID, id2::AbstractID) = get_id(id1) < get_id(id2)
Base.:(>)(id1::AbstractID, id2::AbstractID) = get_id(id1) > get_id(id2)
Base.isless(id1::AbstractID, id2::AbstractID) = id1 < id2
Base.convert(::Type{ID}, i::Int) where {ID<:AbstractID} = ID(i)
Base.copy(id::ID) where {ID<:AbstractID} = ID(get_id(id))

valid_id(id::AbstractID) = get_id(id) > -1

"""
    abstract type AbstractTreeNode{E,ID}

E is element type, ID is id type
"""
abstract type AbstractTreeNode{ID} end
_id_type(n::AbstractTreeNode{ID}) where {ID} = ID
node_id(node::AbstractTreeNode) = node.id
get_parent(n::AbstractTreeNode) = n.parent
get_children(n::AbstractTreeNode) = n.children
has_child(parent::AbstractTreeNode, child::AbstractTreeNode) =
    haskey(get_children(parent), node_id(child))
has_parent(child::AbstractTreeNode, parent::AbstractTreeNode) = get_parent(child) == parent

function rem_parent!(child::AbstractTreeNode)
    delete!(get_children(get_parent(child)), node_id(child))
    child.parent = child
end

function set_parent!(child::AbstractTreeNode, parent::AbstractTreeNode)
    @assert !(child === parent)
    rem_parent!(child)
    get_children(parent)[child.id] = child
    child.parent = parent
    return true
end

"""
    get_root_node(n::AbstractTreeNode{E,ID}) where {E,ID}

Return the root node of a tree
"""
function get_root_node(n::AbstractTreeNode)
    # Identify the root of the tree
    ids = Set{AbstractID}()
    node = n
    parent_id = nothing
    while !(parent_id === node_id(node))
        parent_id = node_id(node)
        if parent_id in ids
            throw(ErrorException("Tree is cyclic!"))
        end
        push!(ids, parent_id)
        node = get_parent(node)
    end
    return node
end

"""
    validate_tree(n::AbstractTreeNode)

Ensure that the subtree of n is in fact a tree--no cycles, and no duplicate ids
"""
function validate_sub_tree(n::N) where {ID,N<:AbstractTreeNode{ID}}
    # Breadth-first search
    node = n
    frontier = Dict{AbstractID,AbstractTreeNode}(node_id(node) => node)
    explored = Set{AbstractID}()
    while !isempty(frontier)
        id, node = pop!(frontier)
        if node_id(node) in explored
            @warn "$node already explored--is there a cycle?"
            return false
        end
        push!(explored, node_id(node))
        for (child_id, child) in get_children(node)
            push!(frontier, child_id => child)
        end
    end
    return true
end

"""
    validate_tree(n::AbstractTreeNode)

Ensure that the transform tree is in fact a tree--no cycles, and no duplicate
ids
"""
function validate_tree(n::AbstractTreeNode)
    node = get_root_node(n)
    validate_sub_tree(node)
end

"""
    validate_embedded_tree(graph,f=v->get_node(graph,v))

Verify that all graph edges are mirrored by the parent-child structure stored in
the nodes.
"""
function validate_embedded_tree(graph, f = v -> get_node(graph, v), early_stop = false)
    valid = true
    for e in edges(graph)
        src = f(edge_source(e))
        dst = f(edge_target(e))
        if !(has_child(src, dst) && has_parent(dst, src))
            @warn "Property \"has_child(src,dst) && has_parent(dst,src)\" does not hold for edge $e"
            valid = false
        end
        early_stop && !valid ? break : nothing
    end
    for u in Graphs.vertices(graph)
        parent = f(u)
        for v in Graphs.vertices(graph)
            child = f(v)
            if has_child(parent, child) || has_parent(child, parent)
                if !(has_child(parent, child) && has_parent(child, parent))
                    if !(parent === child)
                        @warn "has_child($u,$v) = $(has_child(parent,child)) but has_parent($v,$u) = $(has_parent(child,parent))"
                        valid = false
                    end
                end
                if !has_edge(graph, u, v)
                    if !(parent === child)
                        @warn "has_child($u,$v) && has_parent($v,$u) but !has_edge(graph,$u,$v)"
                        valid = false
                    end
                end
            end
            early_stop && !valid ? break : nothing
        end
    end
    return valid
end

"""
	depth_first_search(node::AbstractTreeNode,goal_function,expand_function,
		neighbor_function=outneighbors)

Returns the first vertex satisfying goal_function(graph,v). Only expands v if
expand_function(graph,v) == true.
"""
function depth_first_search(
    node::AbstractTreeNode,
    goal_function,
    expand_function = v -> true,
    explored = Set{_id_type(node)}(),
    skip_first = false,
)
    if goal_function(node)
        if !(skip_first && isempty(explored))
            return node
        end
    end
    push!(explored, node_id(node))
    if expand_function(node)
        for (_, child) in get_children(node)
            if !(node_id(child) in explored)
                retval = depth_first_search(child, goal_function, expand_function, explored)
                if !(retval === nothing)
                    return retval
                end
            end
        end
    end
    return nothing
end

"""
    has_descendant(node::AbstractTreeNode,other::AbstractTreeNode)

Check if `other` is a descendant of `node`.
"""
function has_descendant(node::AbstractTreeNode, other::AbstractTreeNode)
    val = depth_first_search(node, v -> (v === other))
    return !(val === nothing)
end

"""
    has_ancestor(node::AbstractTreeNode,other::AbstractTreeNode)

Check if `other` is an ancestor of `node`.
"""
function has_ancestor(node::AbstractTreeNode, other::AbstractTreeNode)
    has_descendant(other, node)
end

"""
    CachedTreeNode{ID} <: AbstractTreeNode{ID}

Abstract type representing a node with a cached value. Concrete subtypes have a
cached element (accessed via `cached_element(n)`).
"""
abstract type CachedTreeNode{ID} <: AbstractTreeNode{ID} end

mutable struct TreeNode{E,ID} <: CachedTreeNode{ID}
    id::ID
    element::CachedElement{E}
    parent::TreeNode
    children::Dict{ID,TreeNode{E,ID}}
    function TreeNode{E,ID}(e::E) where {E,ID}
        t = new{E,ID}()
        t.id = get_unique_id(ID)
        t.element = CachedElement(e)
        t.parent = t
        t.children = Dict{ID,TreeNode}()
        return t
    end
end

"""
    cached_element(n::CachedTreeNode)   = n.element

Default method for retrieving the cached element of n.
"""
cached_element(n::CachedTreeNode) = n.element
time_stamp(n::CachedTreeNode) = time_stamp(cached_element(n))

"""
    cached_node_up_to_date(n::CachedTreeNode)

Check if n is up to date.
"""
function cached_node_up_to_date(n::CachedTreeNode)
    if is_up_to_date(cached_element(n))
        if !(n === get_parent(n))
            return time_stamp(n) > time_stamp(get_parent(n))
        else
            return true
        end
    end
    return false
end

"""
    set_cached_node_up_to_date!(n::CachedTreeNode,val=true)

Set "up to date" status of n to val.
"""
function set_cached_node_up_to_date!(n::CachedTreeNode, val = true, propagate = true)
    old_val = is_up_to_date(cached_element(n))
    set_up_to_date!(cached_element(n), val)
    # Propagate "outdated" signal up the tree
    # Reduce the amount of wasted forward propagation of "out-of-date" flag
    if propagate && old_val
        for (id, child) in get_children(n)
            set_cached_node_up_to_date!(child, false, propagate)
        end
    end
    return n
end

"""
    propagate_backward!(n::CachedTreeNode,args...)

Propagate information up the tree from n.
"""
function propagate_backward!(n::CachedTreeNode, args...)
    propagate_backward!(n, get_parent(n), args...)
end

"""
    propagate_backward!(child::CachedTreeNode,parent::CachedTreeNode,args...)

Propagate information from child to parent.
"""
propagate_backward!(child::CachedTreeNode, parent::CachedTreeNode, args...) = nothing

"""
    propagate_forward!(n::CachedTreeNode,args...)

Propagate information down the tree from n.
"""
function propagate_forward!(n::CachedTreeNode, args...)
    for (id, child) in get_children(n)
        propagate_forward!(n, child, args...)
    end
    return n
end

"""
    propagate_forward!(parent::CachedTreeNode,child::CachedTreeNode,args...)

Propagate information from parent to child.
"""
propagate_forward!(parent::CachedTreeNode, child::CachedTreeNode, args...) = nothing

"""
    update_element!(n::CachedTreeNode,element,args...)

Update cached element of n, and propagate relevant information forward and
backward via `propagate_forward!` and `propagate_backward!`
"""
function update_element!(n::CachedTreeNode, element, propagate = true, args...)
    update_element!(cached_element(n), element)
    set_cached_node_up_to_date!(n, true)
    if propagate
        propagate_backward!(n, args...)
        propagate_forward!(n, args...)
    end
    return n
end

"""
    get_cached_value!(n::CachedTreeNode)

Return the up to date cached value of n. This triggers a reach back to parent
nodes if necessary.
"""
function get_cached_value!(n::CachedTreeNode)
    if !cached_node_up_to_date(n)
        propagate_forward!(get_parent(n), n)  # retrieve relevant info from parent
    end
    get_element(cached_element(n))
end

abstract type AbstractCustomGraph <: Graphs.AbstractGraph{Int} end

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
const AbstractCustomNDiGraph{N,ID} = AbstractCustomNGraph{Graphs.DiGraph,N,ID}
const AbstractCustomNEDiGraph{N,E,ID} = AbstractCustomNEGraph{Graphs.DiGraph,N,E,ID}

"""
    abstract type AbstractCustomNTree{N,ID} <: AbstractCustomNDiGraph{N,ID}

Abstract custom graph type with tree edge structure.
"""
abstract type AbstractCustomNTree{N,ID} <: AbstractCustomNDiGraph{N,ID} end
abstract type AbstractCustomNETree{N,E,ID} <: AbstractCustomNEDiGraph{N,E,ID} end
const AbstractCustomTree = Union{AbstractCustomNTree,AbstractCustomNETree}

# Common interface
_graph_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID} = G
_node_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID} = N
_id_type(::AbstractCustomNGraph{G,N,ID}) where {G,N,ID} = ID

"""
    get_graph(g::AbstractCustomNGraph{G,N,ID})

return the underlying graph of type `G`.
"""
get_graph(g::AbstractCustomGraph) = g.graph
get_graph(g::Graphs.AbstractGraph) = g

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
get_nodes(g::AbstractCustomNGraph) = g.nodes
Base.zero(g::G) where {G<:AbstractCustomNGraph} = G(graph = _graph_type(g)())
get_vtx(g::AbstractCustomNGraph, v::Int) = v
get_vtx(g::AbstractCustomNGraph{G,N,ID}, id::ID) where {G,N,ID} =
    get(get_vtx_map(g), id, -1)
get_vtx(g::AbstractCustomNGraph{G,N,ID}, node::N) where {G,N,ID} = get_vtx(g, node.id)
get_vtx_id(g::AbstractCustomNGraph, v::Int) = get_vtx_ids(g)[v]
get_node(g::AbstractCustomNGraph, v) = get_nodes(g)[get_vtx(g, v)]

for op in [
    :edgetype,
    :ne,
    :nv,
    :vertices,
    :edges,
    :is_cyclic,
    :topological_sort_by_dfs,
    :is_directed,
    :is_connected,
]
    @eval Graphs.$op(g::AbstractCustomNGraph) = Graphs.$op(get_graph(g))
end
for op in [:outneighbors, :inneighbors, :indegree, :outdegree, :has_vertex]
    @eval Graphs.$op(g::AbstractCustomNGraph, v::Int) = Graphs.$op(get_graph(g), v)
    @eval Graphs.$op(g::AbstractCustomNGraph, id) = Graphs.$op(g, get_vtx(g, id))
end
for op in [:bfs_tree]
    @eval Graphs.$op(g::AbstractCustomNGraph, v::Int; kwargs...) =
        $op(get_graph(g), v; kwargs...)
    @eval Graphs.$op(g::AbstractCustomNGraph, id; kwargs...) =
        $op(get_graph(g), get_vtx(g, id); kwargs...)
end
for op in [:has_edge] #,:add_edge!,:rem_edge!]
    @eval Graphs.$op(s::AbstractCustomNGraph, u, v) =
        $op(get_graph(s), get_vtx(s, u), get_vtx(s, v))
end

"""
    node_id(node)

Return the id of a node. Part of the required node interface for nodes in an
 `AbstractCustomNGraph`.
"""
node_id(node) = node.id

"""
    node_val(node)

Return the value associated with a node. Part of the optional node interface for
nodes in an `AbstractCustomNGraph`.
"""
node_val(node) = node.val

"""
    edge_source(edge)

Returns the ID of the source node of an edge. Part of the required interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_source(edge) = edge.src

"""
    edge_target(edge)

Returns the ID of the target node of an edge. Part of the required interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_target(edge) = edge.dst

"""
    edge_val(edge)

Returns the value associated with an edge. Part of the optional interface for
edges in an `AbstractCustomNEGraph`.
"""
edge_val(edge) = edge.val

function set_vtx_map!(g::AbstractCustomNGraph, node, id, v::Int)
    @assert nv(g) >= v
    get_vtx_map(g)[id] = v
    get_nodes(g)[v] = node
end

function insert_to_vtx_map!(g::AbstractCustomNGraph, node, id, idx::Int = nv(g))
    push!(get_vtx_ids(g), id)
    push!(get_nodes(g), node)
    set_vtx_map!(g, node, id, idx)
end

"""
    replace_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}

Replace the current node associated with `id` with the new node `node`.
"""
function replace_node!(g::AbstractCustomNGraph{G,N,ID}, node::N, id::ID) where {G,N,ID}
    v = get_vtx(g, id)
    @assert v != -1 "node id $(string(id)) is not in graph and therefore cannot be replaced"
    set_vtx_map!(g, node, id, v)
    node
end

"""
    add_node!(g::AbstractCustomNGraph{G,N,ID},node::N,id::ID) where {G,N,ID}

Add `node` to `g` with associated `id`.
"""
function add_node!(g::AbstractCustomNGraph{G,N,ID}, node::N, id::ID) where {G,N,ID}
    @assert !has_vertex(g, id) "Trying to add node $(string(node)) with id $(string(id)) to g, but a node with id $(string(id)) already exists in g"
    add_vertex!(get_graph(g))
    insert_to_vtx_map!(g, node, id, nv(g))
    add_edge_lists!(g)
    node
end

"""
    make_node(g::AbstractCustomNGraph{G,N,ID},val,id)

Construct a node of type `N` from val and id. This method must be implemented
for whatever custom node type is used.
"""
function make_node end
add_node!(g::AbstractCustomNGraph{G,N,ID}, val, id::ID) where {G,N,ID} =
    add_node!(g, make_node(g, val, id), id)
add_node!(g::AbstractCustomNGraph, val, id) = add_node!(g, make_node(g, val, id))
replace_node!(g::AbstractCustomNGraph{G,N,ID}, val, id::ID) where {G,N,ID} =
    replace_node!(g, make_node(g, val, id), id)

"""
    function add_child!(graph,parent,node,id)

add node `child` to `graph` with id `id`, then add edge `parent` → `child`
"""
function add_child!(g::AbstractCustomNGraph{G,N,ID}, parent, child, id) where {G,N,ID}
    n = add_node!(g, child, id)
    if add_edge!(g, parent, id)
        return n
    else
        rem_node!(g, id)
        return nothing
    end
end

"""
    function add_parent!(graph,child,parent,id)

add node `parent` to `graph` with id `id`, then add edge `parent` → `child`
"""
function add_parent!(
    g::AbstractCustomNGraph{G,N,ID},
    child,
    parent::N,
    id::ID,
) where {G,N,ID}
    n = add_node!(g, parent, id)
    if add_edge!(g, id, child)
        return n
    else
        rem_node!(g, id)
        return nothing
    end
end
for op in [:add_child!, :add_parent!]
    @eval $op(g::AbstractCustomNGraph{G,N,ID}, u, v::N) where {G,N,ID} = $op(g, u, v, v.id)
end

"""
    swap_with_end_and_delete!(vec,v)

Replaces `vec[v]` with `last(vec)`, then removes the last element from `vec`
"""
function swap_with_end_and_delete!(vec::Vector, v)
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
    swap_with_end_and_delete!(get_nodes(g), v)
    swap_with_end_and_delete!(get_vtx_ids(g), v)
    delete!(get_vtx_map(g), id)
    if v <= nv(g)
        get_vtx_map(g)[get_vtx_ids(g)[v]] = v
    end
    delete_from_edge_lists!(g, v)  # no effect except for AbstractCustomNEGraph
    g
end
rem_node!(g::AbstractCustomNGraph, v) = rem_node!(g, get_vtx_id(g, v))
function rem_nodes!(g::AbstractCustomNGraph, vtxs::Vector)
    node_ids = collect(map(v -> get_vtx_id(g, v), vtxs))
    for id in node_ids
        rem_node!(g, id)
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
in_edges(g::AbstractCustomNEGraph, v) = in_edges(g)[get_vtx(g, v)]
in_edge(g::AbstractCustomNEGraph, v, u) = in_edges(g, v)[get_vtx(g, u)]
function set_edge!(g::AbstractCustomNEGraph{G,N,E,ID}, u, v, edge::E) where {G,N,E,ID}
    out_edges(g, u)[get_vtx(g, v)] = edge
    in_edges(g, v)[get_vtx(g, u)] = edge
    edge
end

"""
    out_edges(g::AbstractCustomNEGraph{G,N,E,ID})

Returns an integer-indexed forward adjacency list `fadj` (e.g.,
`fadj::Vector{Dict{Int,E}}`) such that `fadj[u::Int][v::Int]` contains the
custom edge associated with `u → v`.
"""
out_edges(g::AbstractCustomNEGraph) = g.outedges
out_edges(g::AbstractCustomNEGraph, v) = out_edges(g)[get_vtx(g, v)]
out_edge(g::AbstractCustomNEGraph, u, v) = out_edges(g, u)[get_vtx(g, v)]

function get_edge(g::AbstractCustomNEGraph, u, v)
    @assert has_edge(g, u, v) "Edge $u → $v does not exist."
    return out_edge(g, u, v)
end

get_edge(g::AbstractCustomNEGraph, edge) = get_edge(g, edge_source(edge), edge_target(edge))
Graphs.has_edge(g::AbstractCustomNEGraph, e) = has_edge(g, edge_source(e), edge_target(e))
add_edge_lists!(g::AbstractCustomNGraph) = g

function add_edge_lists!(g::AbstractCustomNEGraph{G,N,E,ID}) where {G,N,E,ID}
    push!(g.outedges, Dict{Int,E}())
    push!(g.inedges, Dict{Int,E}())
    return g
end

delete_from_edge_lists!(g::AbstractCustomNGraph, v::Int) = g

function delete_from_edge_lists!(g::AbstractCustomNEGraph, v::Int)
    swap_with_end_and_delete!(in_edges(g), v)
    swap_with_end_and_delete!(out_edges(g), v)
    # deleteat!(in_edges(g),v)
    # deleteat!(out_edges(g),v)
    return g
end

function add_custom_edge!(
    g::AbstractCustomNEGraph{G,N,E,ID},
    u,
    v,
    edge::E,
) where {G,N,E,ID}
    if !is_legal_edge(g, u, v, edge)
        @warn "An edge $u → $v is illegal in $g"
        return false
    end
    if add_edge!(get_graph(g), get_vtx(g, u), get_vtx(g, v))
        set_edge!(g, u, v, edge)
        return true
    end
    @warn "Cannot add edge $u → $v. Does an edge already exist?"

    return false
end

add_custom_edge!(g::AbstractCustomNEGraph, edge) =
    add_custom_edge!(g, edge_source(edge), edge_target(edge), edge)
add_custom_edge!(g::AbstractCustomNGraph, edge) =
    add_custom_edge!(g, edge_source(edge), edge_target(edge))
add_custom_edge!(g::AbstractCustomNGraph, u, v) =
    add_edge!(get_graph(g), get_vtx(g, u), get_vtx(g, v))
add_custom_edge!(g::AbstractCustomNGraph, u, v, args...) = add_custom_edge!(g, u, v)
Graphs.add_edge!(g::AbstractCustomGraph, args...) = add_custom_edge!(g, args...)  # no custom edge type here

"""
    make_edge(g::G,u,v,val) where {G}

Construct an edge `u → v` of type `_edge_type(g)` based on val. Default behavior
is to throw an error.
"""
function make_edge(g::G, u, v, val) where {G}
    throw(ErrorException("""
                         MethodError: `make_edge(g,u,v,val)` not implemented.
                         To add an edge to a graph g::$G, either:
                         - add the edge explicitly using `add_edge!(g,edge,[u,v])
                         - make a default constructor for the edge type
                         - implement `GraphUtils.make_edge(g,u,v,val)`
                         """))
end

make_edge(g, u, v) = make_edge(g, u, v, nothing)

function add_custom_edge!(g::AbstractCustomNEGraph, u, v, val)
    add_custom_edge!(g, u, v, make_edge(g, u, v, val))
end

function add_custom_edge!(g::AbstractCustomNEGraph, u, v)
    add_custom_edge!(g, u, v, make_edge(g, u, v))
end

function replace_edge!(g::AbstractCustomNEGraph{G,N,E,ID}, u, v, edge::E) where {G,N,E,ID}
    if has_edge(g, u, v)
        set_edge!(g, u, v, edge)
        return true
    end
    @warn "graph does not have edge $u → $v"
    return false
end

replace_edge!(g::AbstractCustomNEGraph, u, v, val) =
    replace_edge!(g, u, v, make_edge(g, u, v, val))

function replace_edge!(
    g::AbstractCustomNEGraph{G,N,E,ID},
    old_edge,
    edge::E,
) where {G,N,E,ID}
    replace_edge!(g, edge_source(old_edge), edge_target(old_edge), edge)
end

function delete_edge!(g::AbstractCustomNEGraph, u, v)
    if rem_edge!(get_graph(g), get_vtx(g, u), get_vtx(g, v))
        delete!(out_edges(g, u), get_vtx(g, v))
        delete!(in_edges(g, v), get_vtx(g, u))
        return true
    end
    @warn "Cannot remove edge $u → $v. Does it exist?"
    return false
end

delete_edge!(g::AbstractCustomNGraph, u, v) =
    rem_edge!(get_graph(g), get_vtx(g, u), get_vtx(g, v))
delete_edge!(g, edge) = delete_edge!(g, edge_source(edge), edge_target(edge))
Graphs.rem_edge!(g::AbstractCustomGraph, args...) = delete_edge!(g, args...)

# Tree interface
get_parent(g::AbstractCustomTree, v) = get(inneighbors(g, v), 1, -1)
is_legal_edge(g, u, v) = true
is_legal_edge(g, u, v, e) = is_legal_edge(g, u, v)
is_legal_edge(g::AbstractCustomTree, u, v) =
    !(has_vertex(g, get_parent(g, v)) || get_vtx(g, u) == get_vtx(g, v))

function add_custom_edge!(g::AbstractCustomTree, u, v)
    is_legal_edge(g, u, v)
    if !is_legal_edge(g, u, v)
        return false
    end
    add_edge!(get_graph(g), get_vtx(g, u), get_vtx(g, v))
end

"""
    CustomGraph

An example concrete subtype of `AbstractCustomNGraph`.
"""
@with_kw struct CustomNGraph{G<:AbstractGraph,N,ID} <: AbstractCustomNGraph{G,N,ID}
    graph::G = G()
    nodes::Vector{N} = Vector{N}()
    vtx_map::Dict{ID,Int} = Dict{ID,Int}()
    vtx_ids::Vector{ID} = Vector{ID}()  # maps vertex uid to actual graph node
end

const CustomNDiGraph{N,ID} = CustomNGraph{DiGraph,N,ID}

"""
    CustomNEGraph{G,N,E,ID}

Custom graph type with custom edge and node types.
"""
@with_kw struct CustomNEGraph{G,N,E,ID} <: AbstractCustomNEGraph{G,N,E,ID}
    graph::G = G()
    nodes::Vector{N} = Vector{N}()
    vtx_map::Dict{ID,Int} = Dict{ID,Int}()
    vtx_ids::Vector{ID} = Vector{ID}()
    inedges::Vector{Dict{Int,E}} = Vector{Dict{Int,E}}()
    outedges::Vector{Dict{Int,E}} = Vector{Dict{Int,E}}()
end

const CustomNEDiGraph{N,E,ID} = CustomNEGraph{N,E,ID}

"""
    CustomNTree

An example concrete subtype of `AbstractCustomNTree`.
"""
@with_kw struct CustomNTree{N,ID} <: AbstractCustomNTree{N,ID}
    graph::DiGraph = DiGraph()
    nodes::Vector{N} = Vector{N}()
    vtx_map::Dict{ID,Int} = Dict{ID,Int}()
    vtx_ids::Vector{ID} = Vector{ID}()
end

"""
    CustomNETree{G,N,E,ID}

Custom tree type with custom edge and node types.
"""
@with_kw struct CustomNETree{N,E,ID} <: AbstractCustomNETree{N,E,ID}
    graph::DiGraph = DiGraph()
    nodes::Vector{N} = Vector{N}()
    vtx_map::Dict{ID,Int} = Dict{ID,Int}()
    vtx_ids::Vector{ID} = Vector{ID}()
    inedges::Vector{Dict{Int,E}} = Vector{Dict{Int,E}}()
    outedges::Vector{Dict{Int,E}} = Vector{Dict{Int,E}}()
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

CustomNode{N,ID}(id::ID) where {N,ID} = CustomNode{N,ID}(id, N())

function make_node(
    g::AbstractCustomNGraph{G,CustomNode{N,ID},ID},
    val::N,
    id,
) where {G,N,ID}
    _node_type(g)(id, val)
end

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

CustomEdge{E,ID}(id1::ID, id2::ID) where {E,ID} = CustomEdge{E,ID}(id1, id2, E())

function make_edge(
    g::AbstractCustomNEGraph{G,N,CustomEdge{E,ID},ID},
    u,
    v,
    val::E,
) where {G,N,E,ID}
    _edge_type(g)(get_vtx_id(g, get_vtx(g, u)), get_vtx_id(g, get_vtx(g, v)), val)
end

const NGraph{G,N,ID} = CustomNGraph{G,CustomNode{N,ID},ID}
const NEGraph{G,N,E,ID} = CustomNEGraph{G,CustomNode{N,ID},CustomEdge{E,ID},ID}
const NTree{N,ID} = CustomNTree{CustomNode{N,ID},ID}
const NETree{N,E,ID} = CustomNETree{CustomNode{N,ID},CustomEdge{E,ID},ID}
for T in (:CustomNGraph, :CustomNTree)
    @eval begin
        function Base.convert(
            ::Type{X},
            g::AbstractCustomNGraph{G,N,ID},
        ) where {G,N,ID,X<:$T}
            X(
                deepcopy(get_graph(g)),
                deepcopy(get_nodes(g)),
                deepcopy(get_vtx_map(g)),
                deepcopy(get_vtx_ids(g)),
            )
        end
    end
end
for T in (:CustomNEGraph, :CustomNETree)
    @eval begin
        function Base.convert(
            ::Type{X},
            g::AbstractCustomNEGraph{G,N,E,ID},
        ) where {G,N,E,ID,X<:$T}
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

"""
	matches_template(template,node)

Checks if a candidate `node` satisfies the criteria encoded by `template`.
"""
matches_template(template::Type{T}, node::Type{S}) where {T,S} = S <: T
matches_template(template::Type{T}, node::S) where {T,S} = S <: T
matches_template(template, node) = matches_template(typeof(template), node)
matches_template(template::Tuple, node) = any(map(t -> matches_template(t, node), template))
matches_template(template::Type{T}, n::CustomNode) where {T} =
    matches_template(template, node_val(n))

"""
	depth_first_search(graph,v,goal_function,expand_function,
		neighbor_function=outneighbors)

Returns the first vertex satisfying goal_function(graph,v). Only expands v if
expand_function(graph,v) == true.
"""
function depth_first_search(
    graph,
    v,
    goal_function,
    expand_function,
    neighbor_function = outneighbors,
    explored = falses(nv(graph));
    skip_first = false,
)
    if goal_function(v)
        if !(skip_first && sum(explored) == 0)
            return v
        end
    end
    explored[v] = true
    if expand_function(v)
        for vp in neighbor_function(graph, v)
            if !explored[vp]
                u = depth_first_search(
                    graph,
                    vp,
                    goal_function,
                    expand_function,
                    neighbor_function,
                    explored,
                )
                if has_vertex(graph, u)
                    return u
                end
            end
        end
    end
    return -1
end

"""
	has_path(graph,v,v2)

Return true if `graph` has a path from `v` to `v2`
"""
function has_path(graph, v, v2)
    vtx = depth_first_search(graph, v, vtx -> vtx == v2, vtx -> true, outneighbors)
    return vtx > 0
end

"""
	node_iterator(graph,it)

Wraps an iterator over ids or vertices to return the corresponding node at each
iteration.
"""
node_iterator(graph, it) = transform_iter(v -> get_node(graph, v), it)

"""
	filtered_topological_sort(graph,template)

Iterator over nodes that match template.
"""
function filtered_topological_sort(graph, template)
    Base.Iterators.filter(
        n -> matches_template(template, n),
        # transform_iter(v->get_node(graph,v), topological_sort_by_dfs(graph))
        node_iterator(graph, topological_sort_by_dfs(graph)),
    )
end

"""
    transplant!(graph,old_graph,id)

Share node with `id` in `old_graph` to `graph` with the same id.
"""
function transplant!(graph, old_graph, id)
    add_node!(graph, get_node(old_graph, id), id)
end

"""
    backup_descendants(g::AbstractCustomNGraph{G,N,ID},template)

Return a dictionary mapping each node's id to the id of it's closest descendant
matching `template`.
"""
backup_descendants(g, f) = _backup_descendants(g, f, get_graph(g))
backup_ancestors(g, f) = _backup_descendants(g, f, reverse(get_graph(g)))
function _backup_descendants(
    g::AbstractCustomNGraph{G,N,ID},
    f,
    graph = get_graph(g),
) where {G,N,ID}
    descendant_map = Dict{ID,Union{ID,Nothing}}()
    for v in reverse(topological_sort_by_dfs(graph))
        node = get_node(g, v)
        if f(node)
            descendant_map[node_id(node)] = node_id(node)
        elseif is_terminal_node(graph, v)
            descendant_map[node_id(node)] = nothing
        else
            descendant_map[node_id(node)] = nothing
            for vp in outneighbors(graph, v)
                id = get!(descendant_map, get_vtx_id(g, vp), nothing)
                if !(id === nothing)
                    descendant_map[node_id(node)] = id
                end
            end
        end
    end
    descendant_map
end

"""
	get_biggest_tree(graph,dir=:in)

Return the root/terminal vertex corresponding to the root of the largest tree in
the graph.
"""
function get_biggest_tree(graph, dir = :in)
    if dir == :in
        leaves = collect(get_all_terminal_nodes(graph))
    else
        leaves = collect(get_all_root_nodes(graph))
    end
    v = argmax(map(v -> ne(bfs_tree(graph, v; dir = dir)), leaves))
    leaves[v]
end

"""
	collect_subtree(graph,v,dir=:out)

Return a set of all nodes in the subtree of `graph` starting from `v` in
direction `dir`.
"""
function collect_subtree(graph, v, dir = :out, keep = true)
    descendants = Set{Int}()
    for e in edges(bfs_tree(graph, v; dir = dir))
        push!(descendants, e.dst)
    end
    if keep
        push!(descendants, v)
    end
    descendants
end

collect_subtree(graph, vec::AbstractVector{Int}, args...) =
    collect_subtree(graph, Set{Int}(vec), args...)

function collect_subtree(graph, starts::Set{Int}, dir = :out, keep = true)
    frontier = Set{Int}(starts)
    explored = Set{Int}()
    f = dir == :out ? outneighbors : inneighbors
    while !isempty(frontier)
        v = pop!(frontier)
        push!(explored, v)
        for vp in f(graph, v)
            if !(vp in explored)
                push!(frontier, vp)
            end
        end
    end
    if !(keep == true)
        setdiff!(explored, starts)
    end
    return explored
end

"""
	collect_descendants(graph,v) = collect_subtree(graph,v,:out)
"""
collect_descendants(graph, v, keep = false) = collect_subtree(graph, v, :out, keep)

"""
	collect_ancestors(graph,v) = collect_subtree(graph,v,:in)
"""
collect_ancestors(graph, v, keep = false) = collect_subtree(graph, v, :in, keep)

"""
	required_predecessors(node)

Identifies the types (and how many) of required predecessors to `node`
Return type: `Dict{DataType,Int}`
"""
function required_predecessors end

"""
	required_successors(node)

Identifies the types (and how many) of required successors to `node`
Return type: `Dict{DataType,Int}`
"""
function required_successors end

"""
	eligible_predecessors(node)

Identifies the types (and how many) of eligible predecessors to `node`
Return type: `Dict{DataType,Int}`
"""
function eligible_predecessors end

"""
	eligible_successors(node)

Identifies the types (and how many) of eligible successors to `node`
Return type: `Dict{DataType,Int}`
"""
function eligible_successors end

"""
	num_required_predecessors(node)

Returns the total number of required predecessors to `node`.
"""
function num_required_predecessors(node)
    n = 1
    for (key, val) in required_predecessors(node)
        n += val
    end
    n
end

"""
	num_required_successors(node)

Returns the total number of required successors to `node`.
"""
function num_required_successors(node)
    n = 1
    for (key, val) in required_successors(node)
        n += val
    end
    n
end

"""
	num_eligible_predecessors(node)

Returns the total number of eligible predecessors to `node`.
"""
function num_eligible_predecessors(node)
    n = 1
    for (key, val) in eligible_predecessors(node)
        n += val
    end
    n
end

"""
	num_eligible_successors(node)

Returns the total number of eligible successors to `node`.
"""
function num_eligible_successors(node)
    n = 1
    for (key, val) in eligible_successors(node)
        n += val
    end
    n
end

"""
	validate_edge(n1,n2)

For an edge (n1) --> (n2), checks whether the edge is legal and the nodes
"agree".
"""
function validate_edge(a, b)
    valid = false
    for (key, val) in eligible_successors(a)
        if matches_template(key, b) && val >= 1
            valid = true
        end
    end
    for (key, val) in eligible_predecessors(b)
        if matches_template(key, a) && val >= 1
            valid = valid && true
            return valid
        end
    end
    return false
end

validate_edge(n1::CustomNode, n2) = validate_edge(node_val(n1), n2)
validate_edge(n1::CustomNode, n2::CustomNode) = validate_edge(n1, node_val(n2))
for op in [
    :required_successors,
    :required_predecessors,
    :eligible_successors,
    :eligible_predecessors,
]
    @eval $op(n::CustomNode) = $op(node_val(n))
end

# Extend the validation interface to allow dispatch on graph type
for op in [
    :required_successors,
    :required_predecessors,
    :eligible_successors,
    :eligible_predecessors,
    :num_required_successors,
    :num_required_predecessors,
    :num_eligible_successors,
    :num_eligible_predecessors,
]
    @eval $op(graph, node) = $op(node)
end
for op in [:validate_edge]
    @eval $op(graph, n1, n2) = $op(n1, n2)
end

function validate_neighborhood(g, v)
    n = get_node(g, v)
    try
        for (d, list, required, eligible) in [
            (
                :out,
                outneighbors(g, v),
                required_successors(g, n),
                eligible_successors(g, n),
            ),
            (
                :in,
                inneighbors(g, v),
                required_predecessors(g, n),
                eligible_predecessors(g, n),
            ),
        ]
            for vp in list
                np = get_node(g, vp)
                has_match = false
                for k in keys(required)
                    if matches_template(k, np)
                        required[k] -= 1
                        @assert required[k] >= 0 "Node $v has too many $(string(d))neighbors of type $k"
                        has_match = true
                        break
                    end
                end
                for k in keys(eligible)
                    if matches_template(k, np)
                        eligible[k] -= 1
                        @assert eligible[k] >= 0 "Node $v has too many $(string(d))neighbors of type $k"
                        has_match = true
                        break
                    end
                end
                @assert has_match "Node $vp should not be an $(string(d))neighbor of node $v"
            end
        end
    catch e
        if isa(e, AssertionError)
            bt = catch_backtrace()
            showerror(stdout, e, bt)
        else
            rethrow(e)
        end
        return false
    end
    return true
end

function validate_graph(g::AbstractCustomGraph)
    try
        for e in edges(g)
            node1 = get_node(g, e.src)
            node2 = get_node(g, e.dst)
            @assert(
                validate_edge(g, node1, node2),
                string(" INVALID EDGE: ", string(node1), " --> ", string(node2))
            )
        end
        for v in Graphs.vertices(g)
            if !validate_neighborhood(g, v)
                return false
            end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            showerror(stdout, e, bt)
        else
            rethrow(e)
        end
        return false
    end
    return true
end

abstract type AbstractBFSIterator end

Base.IteratorSize(::AbstractBFSIterator) = Base.SizeUnknown()
Base.IteratorEltype(::AbstractBFSIterator) = Base.HasEltype()
Base.eltype(::AbstractBFSIterator) = Int

function Base.iterate(iter::AbstractBFSIterator, v = nothing)
    if !(v === nothing)
        update_iterator!(iter, v)
    end
    if isempty(iter)
        return nothing
    end
    vp = pop!(iter)
    return vp, vp
end

@with_kw struct BFSIterator{G} <: AbstractBFSIterator
    graph::G = Graphs.DiGraph()
    frontier::Set{Int} = get_all_root_nodes(graph)
    next_frontier::Set{Int} = Set{Int}()
    explored::Vector{Bool} = _indicator_vec(nv(graph), frontier)
    replace::Bool = false  # if true, allow nodes to reused
end

BFSIterator(graph) = BFSIterator(graph = graph)
BFSIterator(graph, frontier) = BFSIterator(graph = graph, frontier = frontier)
Base.pop!(iter::BFSIterator) = pop!(iter.frontier)
Base.isempty(iter::BFSIterator) = isempty(iter.frontier)

function update_iterator!(iter::BFSIterator, v)
    iter.explored[v] = true
    for vp in outneighbors(iter.graph, v)
        if iter.replace || !iter.explored[vp]
            push!(iter.next_frontier, vp)
            iter.explored[vp] = true
        end
    end
    if isempty(iter.frontier)
        union!(iter.frontier, iter.next_frontier)
        empty!(iter.next_frontier)
    end
end

@with_kw struct SortedBFSIterator{G} <: AbstractBFSIterator
    graph::G = Graphs.DiGraph()
    frontier::Vector{Int} = sort(collect(get_all_root_nodes(graph)))
    next_frontier::Vector{Int} = Vector{Int}()
    explored::Vector{Bool} = _indicator_vec(nv(graph), frontier)
    replace::Bool = false  # if true, allow nodes to reused
end

SortedBFSIterator(graph) = SortedBFSIterator(graph = graph)
SortedBFSIterator(graph, frontier) = SortedBFSIterator(graph = graph, frontier = frontier)

function Base.pop!(iter::SortedBFSIterator)
    v = popfirst!(iter.frontier)
end

Base.isempty(iter::SortedBFSIterator) = isempty(iter.frontier)

function update_iterator!(iter::SortedBFSIterator, v)
    iter.explored[v] = true
    for vp in outneighbors(iter.graph, v)
        if iter.replace || !iter.explored[vp]
            push!(iter.frontier, vp)
            iter.explored[vp] = true
        end
    end
end

### Graph Plotting ###

@with_kw struct BFS_state
    d::Int = 0  # current depth
    w::Int = 0  # current width
    d_max::Int = 0  # max depth so far
end

function leaf_case!(G, v, s::BFS_state)
    set_prop!(G, v, :height, 0)  # distance to leaf
    set_prop!(G, v, :depth, s.d)  # distance to root
    set_prop!(G, v, :left, s.w)
    set_prop!(G, v, :width, 1)
    set_prop!(G, v, :right, s.w + 1)
    set_prop!(G, v, :center, (get_prop(G, v, :left) + get_prop(G, v, :right)) / 2.0)
    s = BFS_state(d = s.d, w = s.w + 1, d_max = max(s.d_max, s.d))
end

function initial_branch_case!(G, v, s::BFS_state)
    set_prop!(G, v, :height, 0)  # distance to leaf
    set_prop!(G, v, :left, s.w)
    set_prop!(G, v, :depth, s.d)
    set_prop!(G, v, :edge_count, 0)
    set_prop!(G, v, :center, 0.0)
end

function backup!(G, v, v2, s::BFS_state)
    set_prop!(G, v, :height, max(get_prop(G, v, :height), 1 + get_prop(G, v2, :height)))  # distance to leaf
    set_prop!(G, v, :right, s.w)
    set_prop!(G, v, :width, s.w - get_prop(G, v, :left))
    set_prop!(G, v, :edge_count, 1 + get_prop(G, v, :edge_count))
    c = get_prop(G, v, :center)
    e = get_prop(G, v, :edge_count)
    set_prop!(G, v, :center, c * ((e - 1) / e) + get_prop(G, v2, :center) * (1 / e))
end

step_in(s::BFS_state) = BFS_state(d = s.d + 1, w = s.w, d_max = s.d_max)
step_out(s::BFS_state) = BFS_state(d = s.d - 1, w = s.w, d_max = s.d_max)

function bfs!(G, v, s, visited = Set{Int}(), enforce_visited = false)
    s = step_in(s)
    if indegree(G, v) == 0
        s = leaf_case!(G, v, s)
    else
        initial_branch_case!(G, v, s)
        for v2 in inneighbors(G, v)
            if !(v2 in visited) || !enforce_visited
                s = bfs!(G, v2, s, visited, enforce_visited)
                backup!(G, v, v2, s)
            end
        end
    end
    push!(visited, v)
    return step_out(s)
end

abstract type GraphInfoFeature end
struct ForwardDepth <: GraphInfoFeature end
struct BackwardDepth <: GraphInfoFeature end
struct ForwardWidth <: GraphInfoFeature end
struct BackwardWidth <: GraphInfoFeature end
struct VtxWidth <: GraphInfoFeature end
struct ForwardIndex <: GraphInfoFeature end
struct ForwardCenter <: GraphInfoFeature end
struct BackwardIndex <: GraphInfoFeature end
struct BackwardCenter <: GraphInfoFeature end

"""
    Track the width of the tree above a node, and the parent of that tree
"""
struct ForwardTreeWidth <: GraphInfoFeature end
struct BackwardTreeWidth <: GraphInfoFeature end
initial_value(f::Union{ForwardTreeWidth,BackwardTreeWidth}) = Dict{Int,Int}()

function forward_propagate(::ForwardTreeWidth, G, v, v2, vec)
    for (k, val) in vec[v2]
        vec[v][k] = max(get(vec[v], k, 0), val)
    end
end

function backward_propagate(::BackwardTreeWidth, G, v, v2, vec)
    for (k, val) in vec[v2]
        vec[v][k] = max(get(vec[v], k, 0), val)
    end
end

function forward_accumulate(::ForwardTreeWidth, G, v, vtxs, vec)
    if indegree(G, v) == 0
        vec[v][v] = 1
    end
    return vec[v]
end

function backward_accumulate(::BackwardTreeWidth, G, v, vtxs, vec)
    if outdegree(G, v) == 0
        vec[v][v] = 1
    end
    return vec[v]
end

initial_value(f) = 1.0
initial_value(f::Union{ForwardDepth,BackwardDepth}) = 1
forward_propagate(f, G, v, v2, vec) = vec[v]
backward_propagate(f, G, v, v2, vec) = vec[v]
forward_accumulate(f, G, v, vtxs, vec) = vec[v]
backward_accumulate(f, G, v, vtxs, vec) = vec[v]
backward_propagate(::BackwardDepth, G, v, v2, vec) = max(vec[v], vec[v2] + 1)
forward_propagate(::ForwardDepth, G, v, v2, vec) = max(vec[v], vec[v2] + 1)
forward_propagate(::ForwardWidth, G, v, v2, vec) =
    max(vec[v], vec[v2] / max(1, outdegree(G, v2)))
backward_propagate(::BackwardWidth, G, v, v2, vec) =
    max(vec[v], vec[v2] / max(1, indegree(G, v2)))
forward_accumulate(::ForwardWidth, G, v, vtxs, vec) =
    max(vec[v], sum([0, map(v2 -> vec[v2] / outdegree(G, v2), vtxs)...]))
backward_accumulate(::BackwardWidth, G, v, vtxs, vec) =
    max(vec[v], sum([0, map(v2 -> vec[v2] / indegree(G, v2), vtxs)...]))

function forward_pass!(
    G,
    feats,
    feat_vals = Dict(f => map(v -> initial_value(f), Graphs.vertices(G)) for f in feats),
)
    for v in topological_sort_by_dfs(G)
        for (f, vec) in feat_vals
            vec[v] = forward_accumulate(f, G, v, inneighbors(G, v), vec)
            for v2 in inneighbors(G, v)
                vec[v] = forward_propagate(f, G, v, v2, vec)
            end
        end
    end
    return feat_vals
end

function backward_pass!(
    G,
    feats,
    feat_vals = Dict(f => map(v -> initial_value(f), Graphs.vertices(G)) for f in feats),
)
    for v in reverse(topological_sort_by_dfs(G))
        for (f, vec) in feat_vals
            vec[v] = backward_accumulate(f, G, v, outneighbors(G, v), vec)
            for v2 in outneighbors(G, v)
                vec[v] = backward_propagate(f, G, v, v2, vec)
            end
        end
    end
    return feat_vals
end

function get_graph_layout(
    G,
    feats = [ForwardDepth(), BackwardDepth(), ForwardWidth(), BackwardWidth()];
    enforce_visited = false,
)
    @assert !is_cyclic(G)
    feat_vals = forward_pass!(G, feats)
    feat_vals = backward_pass!(G, feats, feat_vals)
    feat_vals[VtxWidth()] = max.(feat_vals[ForwardWidth()], feat_vals[BackwardWidth()])
    graph = MetaDiGraph(nv(G))
    for e in edges(G)
        add_edge!(graph, e)
    end
    end_vtxs = [v for v in Graphs.vertices(graph) if outdegree(graph, v) == 0]
    s = BFS_state(0, 0, 0)
    for v in end_vtxs
        s = bfs!(graph, v, s, Set{Int}(), enforce_visited)
    end
    for (idx_key, ctr_key, depth_key) in [
        (ForwardIndex(), ForwardCenter(), ForwardDepth()),
        ((BackwardIndex(), BackwardCenter(), BackwardDepth())),
    ]
        vec = feat_vals[depth_key]
        forward_width_counts = map(v -> Int[], 1:maximum(vec))
        forward_idxs = zeros(nv(G))
        backward_idxs = zeros(nv(G))
        for v in topological_sort_by_dfs(G)
            push!(forward_width_counts[vec[v]], v)
        end
        for vec in forward_width_counts
            sort!(vec, by = v -> get_prop(graph, v, :left))
            for (i, v) in enumerate(vec)
                if i > 1
                    v2 = vec[i-1]
                    forward_idxs[v] += forward_idxs[v2] + feat_vals[VtxWidth()][v2]
                end
                if indegree(G, v) > 0
                    min_idx = minimum(map(v2 -> forward_idxs[v2], inneighbors(G, v)))
                    forward_idxs[v] = max(forward_idxs[v], min_idx)
                    for v2 in vec[1:i-1]
                        forward_idxs[v] = max(
                            forward_idxs[v],
                            forward_idxs[v2] + feat_vals[VtxWidth()][v2],
                        )
                    end
                end
            end
        end
        feat_vals[idx_key] = forward_idxs
        feat_vals[ctr_key] = forward_idxs .+ 0.5 * feat_vals[VtxWidth()]
    end
    feat_vals
end

# Node plotting utilities
_title_string(n::Int) = string(n)
_title_string(n) = ""
_subtitle_string(n) = ""
_node_color(n) = "red"
_node_bg_color(n) = "white"
_text_color(n) = _node_color(n)
_node_shape(n, t = 0.1) = Compose.circle(0.5, 0.5, 0.5 - t / 2)
_title_text_scale(n) = 0.6
_subtitle_text_scale(n) = 0.2
for op in (:_node_shape, :_node_color, :_subtitle_string, :_text_color, :_title_string)
    @eval $op(graph::AbstractGraph, v, args...) = $op(v, args...)
end

function draw_node(
    n;
    t = 0.1,  # line thickness
    title_scale = _title_text_scale(n),
    subtitle_scale = _subtitle_text_scale(n),
    bg_color = _node_bg_color(n),
    text_color = _text_color(n),
    node_color = _node_color(n),
    title_text = _title_string(n),
    subtitle_text = _subtitle_string(n),
)
    title_y = 0.5
    if !isempty(subtitle_text)
        title_y -= subtitle_scale / 2
    end
    subtitle_y = title_y + (title_scale) / 2
    Compose.compose(
        context(),
        (
            context(),
            Compose.text(0.5, title_y, title_text, hcenter, vcenter),
            Compose.fill(text_color),
            fontsize(title_scale * min(w, h)),
        ),
        (
            context(),
            Compose.text(0.5, subtitle_y, subtitle_text, hcenter, vcenter),
            Compose.fill(text_color),
            Compose.fontsize(subtitle_scale * min(w, h)),
        ),
        (
            context(),
            _node_shape(n, t),
            fill(bg_color),
            Compose.stroke(node_color),
            Compose.linewidth(t * w),
        ),
    )
end

draw_node(graph, v; kwargs...) = draw_node(v; kwargs...)

function default_draw_node(G, v; fg_color = "red", bg_color = "white", stroke_width = 0.1)
    draw_node(G, v; t = stroke_width)
end

default_draw_edge(G, v1, v2, pt1, pt2; fg_color = "blue", stroke_width = 0.01) = (
    context(),
    Compose.line([pt1, pt2]),
    Compose.stroke(fg_color),
    Compose.linewidth(stroke_width * w),
)

function draw_ortho_edge(G, v1, v2, pt1, pt2; fg_color = "blue", stroke_width = 0.01)
    ymid = (pt1[2] + pt2[2]) / 2
    (
        context(),
        Compose.line([pt1, (pt1[1], ymid), (pt2[1], ymid), pt2]),
        Compose.stroke(fg_color),
        Compose.linewidth(stroke_width * w),
    )
end

"""
    display_graph(graph;kwargs...)

Plot a graph.
"""
function display_graph(
    graph;
    draw_node_function = (G, v) -> default_draw_node(G, v),
    draw_edge_function = (G, v, v2, pt1, pt2) -> default_draw_edge(G, v, v2, pt1, pt2),
    grow_mode = :from_bottom,  # :from_left, :from_bottom, :from_top,
    align_mode = :leaf_aligned,  # :root_aligned
    node_size = (0.75, 0.75),
    scale = 1.0,
    pad = (0.5, 0.5),
    aspect_stretch = (1.0, 1.0),
    bg_color = "white",
    enforce_visited = false,
)
    feat_vals = get_graph_layout(graph; enforce_visited = enforce_visited)
    # Check alignment mode
    if align_mode == :leaf_aligned
        x = feat_vals[ForwardDepth()]
        y = feat_vals[ForwardCenter()]
    elseif align_mode == :root_aligned
        x = feat_vals[BackwardDepth()]
        x = 1 + maximum(x) .- x
        y = feat_vals[BackwardCenter()]
    elseif align_mode == :split_aligned
        x = feat_vals[ForwardDepth()]
        y = feat_vals[ForwardCenter()]
        # Start with the root node, which is the max x value for forward_depth
        root_node = argmax(x)
        parents = [root_node]
        temp_parents = []
        while !isempty(parents)
            for parent in parents
                for v in inneighbors(graph, parent)
                    push!(temp_parents, v)
                    for vc in outneighbors(graph, v)
                        if vc == parent
                            continue
                        end
                        x[vc] = x[parent]
                    end
                    x[v] = x[parent] - 1
                end
            end
            parents = temp_parents
            temp_parents = []
        end
    else
        throw(ErrorException("Unknown align_mode $align_mode"))
    end
    # Check growth direction
    if grow_mode == :from_left
        # Do nothing
    elseif grow_mode == :from_right
        x = -x
    elseif grow_mode == :from_bottom
        x, y = y, -x
    elseif grow_mode == :from_top
        x, y = y, x
    else
        throw(ErrorException("Unknown grow_mode $grow_mode"))
    end
    # Ensure positive and shift to be on the canvas (how to shift the canvas instead?)
    x *= aspect_stretch[1]
    y *= aspect_stretch[2]
    x = x .- minimum(x) .+ node_size[1] / 2  # .+ pad[1]
    y = y .- minimum(y) .+ node_size[2] / 2  # .+ pad[2]
    context_size = (maximum(x) + node_size[1] / 2, maximum(y) + node_size[2] / 2)
    canvas_size = (context_size[1] .+ 2 * pad[1], context_size[2] .+ 2 * pad[2])
    set_default_graphic_size((scale * canvas_size[1])cm, (scale * canvas_size[2])cm)
    node_context(a, b, s = node_size) = context(
        (a - s[1] / 2),
        (b - s[2] / 2),
        s[1],
        s[2],
        units = UnitBox(0.0, 0.0, 1.0, 1.0),
    )
    edge_context(a, b) = context(a, b, 1.0, 1.0, units = UnitBox(0.0, 0.0, 1.0, 1.0))
    nodes = map(
        v -> (node_context(x[v], y[v]), draw_node_function(graph, v)),
        Graphs.vertices(graph),
    )
    lines = []
    for e in edges(graph)
        dx = x[e.dst] - x[e.src]
        dy = y[e.dst] - y[e.src]
        push!(
            lines,
            (
                edge_context(x[e.src], y[e.src]),
                draw_edge_function(graph, e.src, e.dst, (0.0, 0.0), (dx, dy)),
            ),
        )
    end
    Compose.compose(
        context(units = UnitBox(0.0, 0.0, canvas_size...)),
        (
            context(
                pad[1],
                pad[2],
                context_size...,
                units = UnitBox(0.0, 0.0, context_size...),
            ),
            nodes...,
            lines...,
        ),
        Compose.compose(context(), rectangle(), fill(bg_color)),
    )
end
