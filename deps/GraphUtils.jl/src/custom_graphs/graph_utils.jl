################################################################################
################################### Utilities ##################################
################################################################################

export
	BFSIterator,
	SortedBFSIterator,
	update_iterator!,
	depth_first_search,
	node_iterator,
	filtered_topological_sort,
    get_nodes_of_type,
    transplant!,
    forward_pass!,
	backward_pass!,
	backup_descendants,
	backup_ancestors,
	get_biggest_tree,
	collect_subtree,
	collect_descendants,
	collect_ancestors,

    validate_edge,

    matches_template,
	required_predecessors,
	required_successors,
	num_required_predecessors,
	num_required_successors,
    eligible_successors,
    eligible_predecessors,
	num_eligible_predecessors,
	num_eligible_successors


"""
	matches_template(template,node)

Checks if a candidate `node` satisfies the criteria encoded by `template`.
"""
matches_template(template::Type{T},node::Type{S}) where {T,S} = S<:T
matches_template(template::Type{T},node::S) where {T,S} = S<:T
matches_template(template,node) = matches_template(typeof(template),node)
matches_template(template::Tuple,node) = any(map(t->matches_template(t,node), template))

matches_template(template::Type{T},n::CustomNode) where {T} = matches_template(template,node_val(n))
# matches_template(template::Type{T},n::CustomNode{N,ID}) where {T,N,ID} = matches_template(template,N)

abstract type AbstractBFSIterator end
Base.IteratorSize(::AbstractBFSIterator) = Base.SizeUnknown()
Base.IteratorEltype(::AbstractBFSIterator) = Base.HasEltype()
Base.eltype(::AbstractBFSIterator) = Int
function Base.iterate(iter::AbstractBFSIterator,v=nothing)
	if !(v === nothing)
		update_iterator!(iter,v)
	end
	if isempty(iter)
		return nothing
	end
	vp = pop!(iter)
	return vp, vp
end

function _indicator_vec(N,idxs)
	vec = zeros(Bool,N)
	for i in idxs
		vec[i] = true
	end
	vec
end
@with_kw struct BFSIterator{G} <: AbstractBFSIterator
	graph::G 				= DiGraph()
	frontier::Set{Int} 		= get_all_root_nodes(graph)
	next_frontier::Set{Int}	= Set{Int}()
	explored::Vector{Bool}  = _indicator_vec(nv(graph),frontier)
	replace::Bool			= false # if true, allow nodes to reused
end
BFSIterator(graph) = BFSIterator(graph=graph)
BFSIterator(graph,frontier) = BFSIterator(graph=graph,frontier=frontier)
Base.pop!(iter::BFSIterator) = pop!(iter.frontier)
Base.isempty(iter::BFSIterator) = isempty(iter.frontier)
function update_iterator!(iter::BFSIterator,v)
	iter.explored[v] = true
	for vp in outneighbors(iter.graph,v)
		if iter.replace || !iter.explored[vp]
			push!(iter.next_frontier,vp)
			iter.explored[vp] = true
		end
	end
	if isempty(iter.frontier)
		union!(iter.frontier, iter.next_frontier)
		empty!(iter.next_frontier)
	end
end

@with_kw struct SortedBFSIterator{G} <: AbstractBFSIterator
	graph::G 				= DiGraph()
	frontier::Vector{Int}   = sort(collect(get_all_root_nodes(graph)))
	next_frontier::Vector{Int} = Vector{Int}()
	explored::Vector{Bool}  = _indicator_vec(nv(graph),frontier)
	replace::Bool			= false # if true, allow nodes to reused
end
SortedBFSIterator(graph) = SortedBFSIterator(graph=graph)
SortedBFSIterator(graph,frontier) = SortedBFSIterator(graph=graph,frontier=frontier)
function Base.pop!(iter::SortedBFSIterator)
	v = popfirst!(iter.frontier)
end
Base.isempty(iter::SortedBFSIterator) = isempty(iter.frontier)
function update_iterator!(iter::SortedBFSIterator,v)
	iter.explored[v] = true
	for vp in outneighbors(iter.graph,v)
		if iter.replace || !iter.explored[vp]
			push!(iter.frontier,vp)
			iter.explored[vp] = true
		end
	end
end


"""
	depth_first_search(graph,v,goal_function,expand_function,
		neighbor_function=outneighbors)

Returns the first vertex satisfying goal_function(graph,v). Only expands v if 
expand_function(graph,v) == true.
"""
function depth_first_search(graph,v,
		goal_function,
		expand_function,
		neighbor_function=outneighbors,
		explored=falses(nv(graph));
		skip_first=false,
		)
	if goal_function(v)
		if !(skip_first && sum(explored) == 0)
			return v
		end
	end
	explored[v] = true
	if expand_function(v)
		for vp in neighbor_function(graph,v)
			if !explored[vp]
				u = depth_first_search(graph,vp,
					goal_function,
					expand_function,
					neighbor_function,
					explored)
				if has_vertex(graph,u)
					return u
				end
			end
		end
	end
	return -1
end

"""
	dfs_check_cycle(graph,v,neighbor_func=inneighbors)

Check if `graph` contains a cycle with `v` in it
"""
function dfs_check_cycle(graph,v,neighbor_func=inneighbors)
	vtx = depth_first_search(graph,v,
		vtx->vtx==v,
		vtx->true,
		neighbor_func,
		falses(nv(graph)),
		skip_first=true,
		)
	has_vertex(graph,vtx)
end

"""
	has_path(graph,v,v2)

Return true if `graph` has a path from `v` to `v2`
"""
function has_path(graph,v,v2)
	vtx = depth_first_search(graph,v,vtx->vtx==v2,vtx->true,outneighbors)
	return vtx > 0
end


"""
	node_iterator(graph,it)

Wraps an iterator over ids or vertices to return the corresponding node at each
iteration.
"""
node_iterator(graph,it) = transform_iter(v->get_node(graph,v),it)

"""
	filtered_topological_sort(graph,template)

Iterator over nodes that match template.
"""
function filtered_topological_sort(graph,template)
	Base.Iterators.filter(n->matches_template(template,n),
		# transform_iter(v->get_node(graph,v), topological_sort_by_dfs(graph))
		node_iterator(graph, topological_sort_by_dfs(graph))
		)
end

"""
    transplant!(graph,old_graph,id)

Share node with `id` in `old_graph` to `graph` with the same id.
"""
function transplant!(graph,old_graph,id)
    add_node!(graph,get_node(old_graph,id),id)
end

# get_nodes_of_type(g::AbstractCustomNGraph,T) = Dict(id=>get_node(g, id) for id in get_vtx_ids(g) if isa(id,T))
# get_nodes_of_type(g::AbstractCustomNGraph,T) = Dict(id=>get_node(g, id) for id in get_vtx_ids(g) if matches_template(T,id))
# get_nodes_of_type(g::AbstractCustomNGraph,T) = Dict(get_vtx_id(g,v)=>node for (v,node) in enumerate(get_nodes(g)) if matches_template(T,id))
function get_nodes_of_type(g::AbstractCustomNGraph,T)
	d = Dict()
	for (v,node) in enumerate(get_nodes(g))
		id = get_vtx_id(g,v)
		if matches_template(T,id) || matches_template(T,node)
			d[id] = node
		end
	end
	return d
end

function forward_pass!(g::AbstractCustomNGraph,init_function,update_function)
    init_function(g)
    for v in topological_sort_by_dfs(g)
        update_function(g,v)
    end
    return g
end
function backward_pass!(g::AbstractCustomNGraph,init_function,update_function)
    init_function(g)
    for v in reverse(topological_sort_by_dfs(g))
        update_function(g,v)
    end
    return g
end

"""
    backup_descendants(g::AbstractCustomNGraph{G,N,ID},template)

Return a dictionary mapping each node's id to the id of it's closest descendant
matching `template`.
"""
backup_descendants(g,f) = _backup_descendants(g,f,get_graph(g))
backup_ancestors(g,f) = _backup_descendants(g,f,reverse(get_graph(g)))
function _backup_descendants(g::AbstractCustomNGraph{G,N,ID},f,
		graph=get_graph(g),
		) where {G,N,ID}
    descendant_map = Dict{ID,Union{ID,Nothing}}()
    for v in reverse(topological_sort_by_dfs(graph))
		node = get_node(g,v)
        if f(node)
            descendant_map[node_id(node)] = node_id(node)
		elseif is_terminal_node(graph,v)
            descendant_map[node_id(node)] = nothing
		else
			descendant_map[node_id(node)] = nothing
			for vp in outneighbors(graph,v)
				id = get!(descendant_map,get_vtx_id(g,vp),nothing)
				if !(id === nothing)
					descendant_map[node_id(node)] = id
				end
            end
        end
    end
    descendant_map
end
backup_multiple_descendants(g,f) = _backup_multiple_descendants(g,f,get_graph(g))
backup_multiple_ancestors(g,f) = _backup_multiple_descendants(g,f,reverse(get_graph(g)))
function _backup_multiple_descendants(g::AbstractCustomNGraph{G,N,ID},f,
		graph=get_graph(g),
		) where {G,N,ID}
    descendant_map = Dict{ID,Set{ID}}()
    for v in reverse(topological_sort_by_dfs(graph))
		node = get_node(g,v)
		dset = descendant_map[node_id(node)] = Set{ID}()
        if f(node)
			push!(dset,node_id(node))
		elseif is_terminal_node(graph,v)
		else
			for vp in outneighbors(graph,v)
				union!(dset,descendant_map[get_vtx_id(g,vp)])
            end
        end
    end
    descendant_map
end

forward_cover(g,vtxs,f) = _forward_cover(g,vtxs,f,get_graph(g))
backward_cover(g,vtxs,f) = _forward_cover(g,vtxs,f,reverse(get_graph(g)))
function _forward_cover(g,vtxs,f,graph=get_graph(g))
	iter = SortedBFSIterator(graph,Vector(unique(vtxs)))
	cover = Set{_node_type(g)}()
	while !isempty(iter)
		v = pop!(iter)
		n = get_node(g,v)
		if f(n) && !(v in vtxs)
			push!(cover,n)
		else
			update_iterator!(iter,v)
		end
	end
	cover
end
_forward_cover(g,id::AbstractID,args...) = _forward_cover(g,get_vtx(g,id),args...)

"""
	contract_by_node_type(g,f)

Constructs a new graph of the same type as `g`, whose nodes are formed by those 
for which `f(n) == true`. Edges are contracted so that, e.g. 
	`n1 => n2 => n3 => n4` collapses to `n1 => n4` 
	if  `f(n1) && !(f(n2)) && !(f(n3)) && f(n4) `.
"""
function contract_by_predicate(g,f)
    tg = typeof(g)()
    descendant_map = backup_multiple_descendants(g,f)
    ancestor_map = backup_multiple_ancestors(g,f)
    for n in get_nodes(g)
        if f(n)
            add_node!(tg,n,node_id(n))
        end
    end
    for n in get_nodes(g)
        if !f(n)
			for a in ancestor_map[node_id(n)]
				for d in descendant_map[node_id(n)]
					if !has_edge(tg,a,d)
						add_edge!(tg,a,d)
					end
				end
			end
        end
    end
    tg
end

"""
	get_biggest_tree(graph,dir=:in)

Return the root/terminal vertex corresponding to the root of the largest tree in 
the graph.
"""
function get_biggest_tree(graph,dir=:in)
	if dir==:in
    	leaves = collect(get_all_terminal_nodes(graph))
	else
    	leaves = collect(get_all_root_nodes(graph))
	end
	v = argmax(map(v->ne(bfs_tree(graph,v;dir=dir)),leaves))
	leaves[v]
end

"""
	collect_subtree(graph,v,dir=:out)

Return a set of all nodes in the subtree of `graph` starting from `v` in 
direction `dir`.
"""
function collect_subtree(graph,v,dir=:out,keep=true)
	descendants = Set{Int}()
	for e in edges(bfs_tree(graph,v;dir=dir))
		push!(descendants,e.dst)
	end
	if keep
		push!(descendants,v)
	end
	descendants
end
collect_subtree(graph,vec::AbstractVector{Int},args...) = collect_subtree(graph,Set{Int}(vec),args...)
function collect_subtree(graph,starts::Set{Int},dir=:out,keep=true)
	frontier = Set{Int}(starts)
	explored = Set{Int}()
	f = dir == :out ? outneighbors : inneighbors
	while !isempty(frontier)
		v = pop!(frontier)
		push!(explored,v)
		for vp in f(graph,v)
			if !(vp in explored)
				push!(frontier,vp)
			end
		end
	end
	if !(keep == true)
		setdiff!(explored,starts)
	end
	return explored
end

"""
	collect_descendants(graph,v) = collect_subtree(graph,v,:out)
"""
collect_descendants(graph,v,keep=false) = collect_subtree(graph,v,:out,keep)
"""
	collect_ancestors(graph,v) = collect_subtree(graph,v,:in)
"""
collect_ancestors(graph,v,keep=false) = collect_subtree(graph,v,:in,keep)

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
	for (key,val) in required_predecessors(node)
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
	for (key,val) in required_successors(node)
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
	for (key,val) in eligible_predecessors(node)
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
	for (key,val) in eligible_successors(node)
		n += val
	end
	n
end

illegal_edge_msg(n1,n2) = "Edges from `$(typeof(n1))` → `$(typeof(n2))` are illegal"
illegal_indegree_msg(n,val,lo,hi) = "`$(typeof(n))` nodes should $lo ≦ indegree(n) ≦ $hi, but indegree(n) = val"
illegal_outdegree_msg(n,val,lo,hi) = "`$(typeof(n))` nodes should $lo ≦ outdegree(n) ≦ $hi, but indegree(n) = val"

"""
	validate_edge(n1,n2)

For an edge (n1) --> (n2), checks whether the edge is legal and the nodes
"agree".
"""
# function validate_edge(n1,n2)
#     return false
# end
function validate_edge(a,b)
    valid = false
    for (key,val) in eligible_successors(a)
        if matches_template(key,b) && val >= 1
            valid = true
        end
    end
    for (key,val) in eligible_predecessors(b)
        if matches_template(key,a) && val >= 1
            valid = valid && true
            return valid
        end
    end
    return false
end

validate_edge(n1::CustomNode,n2) = validate_edge(node_val(n1),n2)

validate_edge(n1::CustomNode,n2::CustomNode) = validate_edge(n1,node_val(n2))
for op in [:required_successors,:required_predecessors,:eligible_successors,:eligible_predecessors]
	@eval $op(n::CustomNode) = $op(node_val(n))
end

# extending the validation interface to allow dispatch on graph type
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
	@eval $op(graph,node) = $op(node)
end
for op in [
	:validate_edge,
	]
	@eval $op(graph,n1,n2) = $op(n1,n2)
end

# indegree_bounds(n,g,v) = (0,0)
# outdegree_bounds(n,g,v) = (0,0)
# function validate_indegree(n,g,v)
#     lo,hi = indegree_bounds(n,g,v)
#     @assert (lo <= indegree(g,v) <= hi) illegal_indegree_msg(n,indegree(g,v),lo,hi)
# end
# function validate_outdegree(n,g,v)
#     lo,hi = outdegree_bounds(n,g,v)
#     @assert (lo <= outdegree(g,v) <= hi) illegal_outdegree_msg(n,outdegree(g,v),lo,hi)
# end

function validate_neighborhood(g,v)
	n = get_node(g,v)
	try
		for (d,list,required,eligible) in [
				(:out,outneighbors(g,v),required_successors(g,n),eligible_successors(g,n)),
				(:in,inneighbors(g,v),required_predecessors(g,n),eligible_predecessors(g,n)),
			]
			for vp in list
				np = get_node(g,vp)
				has_match = false
				for k in keys(required)
					if matches_template(k,np)
						required[k] -= 1
						@assert required[k] >= 0 "Node $v has too many $(string(d))neighbors of type $k"
						has_match = true
						break
					end
				end
				for k in keys(eligible)
					if matches_template(k,np)
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
		if isa(e,AssertionError)
			bt = catch_backtrace()
            showerror(stdout,e,bt)
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
            node1 = get_node(g,e.src)
            node2 = get_node(g,e.dst)
            @assert(validate_edge(g,node1,node2), string(" INVALID EDGE: ", string(node1), " --> ",string(node2)))
        end
        for v in vertices(g)
			if !validate_neighborhood(g,v)
				return false
			end
        end
    catch e
        if typeof(e) <: AssertionError
            bt = catch_backtrace()
            showerror(stdout,e,bt)
        else
            rethrow(e)
        end
        return false
    end
    return true
end

function print_tree_level(io,tree,v,start,f=summary,spacing=" ")
    println(io,start,f(get_node(tree,v)))
    for vp in outneighbors(tree,v)
        print_tree_level(io,tree,vp,string(start,spacing),f,spacing)
    end
end
function Base.print(io::IO,tree::AbstractCustomTree,f=summary,spacing=" ")
    @assert !is_cyclic(tree)
    println(io,typeof(tree))
    for v in get_all_root_nodes(tree)
        print_tree_level(io,tree,v,"",f,spacing)
    end
end


function log_graph_edges(graph,v0=-1;
        show_upstream = true,
        show_downstream = true,
        show_all = true,
    )
    if has_vertex(graph,v0)
        if show_upstream
            @info "upstream from $(summary(get_node(graph,v0))):"
			bfs_graph = reverse(bfs_tree(graph,v0;dir=:in))
			for v in topological_sort_by_dfs(bfs_graph)
				for vp in outneighbors(bfs_graph,v)
					e = Edge(v,vp)
    	        	@info "$(summary(get_node(graph,e.src))) => $(summary(get_node(graph,e.dst))),"
				end
			end
        end
        if show_downstream
            @info "downstream from $(summary(get_node(graph,v0))):"
			bfs_graph = bfs_tree(graph,v0;dir=:out)
			for v in topological_sort_by_dfs(bfs_graph)
				for vp in outneighbors(bfs_graph,v)
					e = Edge(v,vp)
    	        	@info "$(summary(get_node(graph,e.src))) => $(summary(get_node(graph,e.dst))),"
				end
			end
        end
    end
    if show_all
		for v in topological_sort_by_dfs(graph)
			for vp in outneighbors(graph,v)
				e = Edge(v,vp)
            	@info "$(summary(get_node(graph,e.src))) => $(summary(get_node(graph,e.dst))),"
			end
		end
        # for e in edges(graph)
        #     @info "$(summary(get_node(graph,e.src))) => $(summary(get_node(graph,e.dst))),"
        # end
    end
end

