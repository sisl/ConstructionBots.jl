export
    AbstractTreeNode,
    get_parent,
    get_children,
    rem_parent!,
    set_parent!,
    has_child,
    has_parent,
    validate_tree,
    validate_embedded_tree

"""
    abstract type AbstractTreeNode{E,ID}

E is element type, ID is id type
"""
abstract type AbstractTreeNode{ID} end
_id_type(n::AbstractTreeNode{ID}) where {ID} = ID

node_id(node::AbstractTreeNode) = node.id
get_parent(n::AbstractTreeNode) = n.parent
get_children(n::AbstractTreeNode) = n.children
has_child(parent::AbstractTreeNode,child::AbstractTreeNode) = haskey(get_children(parent),node_id(child))
has_parent(child::AbstractTreeNode,parent::AbstractTreeNode) = get_parent(child) == parent
function rem_parent!(child::AbstractTreeNode)
    delete!(get_children(get_parent(child)), node_id(child))
    child.parent = child
end
function set_parent!(child::AbstractTreeNode,parent::AbstractTreeNode)
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
    # identify the root of the tree
    ids = Set{AbstractID}()
    node = n
    parent_id = nothing
    while !(parent_id === node_id(node))
        parent_id = node_id(node)
        if parent_id in ids
            throw(ErrorException("Tree is cyclic!"))
        end
        push!(ids,parent_id)
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
    frontier = Dict{AbstractID,AbstractTreeNode}(node_id(node)=>node)
    explored = Set{AbstractID}()
    while !isempty(frontier)
        id,node = pop!(frontier)
        if node_id(node) in explored
            @warn "$node already explored--is there a cycle?"
            return false
        end
        push!(explored,node_id(node))
        for (child_id,child) in get_children(node)
            push!(frontier,child_id=>child)
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
function validate_embedded_tree(graph,f=v->get_node(graph,v),early_stop=false)
    valid = true 
    for e in edges(graph)
        src = f(edge_source(e))
        dst = f(edge_target(e))
        if !(has_child(src,dst) && has_parent(dst,src))
            @warn "Property \"has_child(src,dst) && has_parent(dst,src)\" does not hold for edge $e"
            valid = false
        end
        early_stop && !valid ? break : nothing
    end
    for u in vertices(graph)
        parent = f(u)
        for v in vertices(graph)
            child = f(v)
            if has_child(parent,child) || has_parent(child,parent)
                if !(has_child(parent,child) && has_parent(child,parent)) 
                    if !(parent === child)
                        @warn "has_child($u,$v) = $(has_child(parent,child)) but has_parent($v,$u) = $(has_parent(child,parent))"
                        valid = false
                    end
                end
                if !has_edge(graph,u,v)
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
function depth_first_search(node::AbstractTreeNode,
		goal_function,
		expand_function=v->true,
		explored=Set{_id_type(node)}(),
		skip_first=false,
		)
	if goal_function(node)
		if !(skip_first && isempty(explored))
			return node
		end
    end
    push!(explored,node_id(node))
	if expand_function(node)
		for (_,child) in get_children(node)
			if !(node_id(child) in explored)
				retval = depth_first_search(child,
					goal_function,
					expand_function,
                    explored)
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
function has_descendant(node::AbstractTreeNode,other::AbstractTreeNode)
    val = depth_first_search(node, v->( v === other))
    return !(val === nothing)
end

"""
    has_ancestor(node::AbstractTreeNode,other::AbstractTreeNode)

Check if `other` is an ancestor of `node`.
"""
function has_ancestor(node::AbstractTreeNode,other::AbstractTreeNode)
    has_descendant(other,node)
end

export
    CachedTreeNode,
    cached_element

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
cached_element(n::CachedTreeNode)   = n.element
time_stamp(n::CachedTreeNode)       = time_stamp(cached_element(n))

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
function set_cached_node_up_to_date!(n::CachedTreeNode,val=true,propagate=true) 
    old_val = is_up_to_date(cached_element(n))
    set_up_to_date!(cached_element(n),val)
    # propagate "outdated" signal up the tree
    if propagate && old_val # reduce the amount of wasted forward propagation of "out-of-date" flag
        for (id,child) in get_children(n)
            set_cached_node_up_to_date!(child,false,propagate)
        end
    end
    return n
end

"""
    propagate_backward!(n::CachedTreeNode,args...)

Propagate information up the tree from n.
"""
function propagate_backward!(n::CachedTreeNode,args...)
    propagate_backward!(n,get_parent(n),args...)
end
"""
    propagate_backward!(child::CachedTreeNode,parent::CachedTreeNode,args...)

Propagate information from child to parent.
"""
propagate_backward!(child::CachedTreeNode,parent::CachedTreeNode,args...) = nothing
"""
    propagate_forward!(n::CachedTreeNode,args...)

Propagate information down the tree from n.
"""
function propagate_forward!(n::CachedTreeNode,args...)
    for (id,child) in get_children(n)
        propagate_forward!(n,child,args...)
    end
    return n
end
"""
    propagate_forward!(parent::CachedTreeNode,child::CachedTreeNode,args...)

Propagate information from parent to child.
"""
propagate_forward!(parent::CachedTreeNode,child::CachedTreeNode,args...) = nothing

"""
    update_element!(n::CachedTreeNode,element,args...)

Update cached element of n, and propagate relevant information forward and 
backward via `propagate_forward!` and `propagate_backward!`
"""
function update_element!(n::CachedTreeNode,element,propagate=true,args...)
    update_element!(cached_element(n),element)
    set_cached_node_up_to_date!(n,true) 
    if propagate
        propagate_backward!(n,args...)
        propagate_forward!(n,args...)
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
        propagate_forward!(get_parent(n),n) # retrieve relevant info from parent
    end
    get_element(cached_element(n))
end