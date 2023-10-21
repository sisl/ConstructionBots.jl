# module Sorting
#
# using LightGraphs
# using Random

export
    topological_sort,
    find_index_in_sorted_array,
    insert_to_sorted_array!,
    linear_interp,
    get_interp_interval,
    resample_array

"""
    topological_sort(G)

Returns a topological sort of the vertices of a graph, with the property that
v1 < v2 iff there is not forward path through the graph from v2 to v1.
G must be non-cyclic. Can handle disjoint graphs.
"""
function topological_sort(G)
    @assert(!is_cyclic(G))
    frontier = Set{Int}(rand(1:nv(G)))
    # explored = Set{Int}()
    explored = fill!(Vector{Bool}(undef,nv(G)), false)
    ordering = []
    while any(explored .!= true)
        while length(frontier) > 0
            v = pop!(frontier)
            tip = true # check whether all downstream nodes are explored
            for v2 in inneighbors(G,v)
                if !explored[v2]
                    tip = false
                    push!(frontier,v2)
                    break
                end
            end
            if tip == true
                # push!(explored,v)
                explored[v] = true
                push!(ordering, v)
                for v2 in outneighbors(G,v)
                    push!(frontier,v2)
                end
            end
        end
        for v in 1:nv(G)
            if !explored[v]
                push!(frontier, v)
            end
        end
    end
    ordering
end

"""
    find_index_in_sorted_array(array, x)

Assumes that array is already sorted. Returns index at which x would need to
be inserted in order to maintain ordering of array. Chooses the smallest
index in the case of a tie.

Looks like  "Base.searchsorted" does the same thing as this.
"""
function find_index_in_sorted_array(array, x)
    A = 0
    C = length(array)+1
    B = max(1,Int(round((A+C) / 2)))
    while C-A > 1
        if x < array[B] || ( !(array[B] < x) && !(x < array[B]))
            A = A
            C = B
            B = Int(ceil((A+C) / 2))
        else
            A = B
            C = C
            B = Int(ceil((A+C) / 2))
        end
    end
    return B
end

"""
    insert_to_sorted_array!(array, x)

Assumes that array is already sorted. Inserts new element x so that
array remains sorted. Requires that Base.isless(a::C,b::C) where
C==typeof(x) be implemented.
"""
function insert_to_sorted_array!(array, x)
    B = find_index_in_sorted_array(array, x)
    insert!(array, B, x)
    array
end

"""
    linear_interpolation
"""
function linear_interp(a,b,t)
    return a*(1-t) + b*t
end
function linear_interp(a,b,t1,t2,t)
    lin_interp(a,b,(t-t1)/(t-t2))
    return a*(1-t) + b*t
end
function get_interp_interval(array,x)
    idx = find_index_in_sorted_array(array, x)
    if idx == 1
        return idx,idx+1
    elseif idx > length(array)
        return max(1,idx-2),idx-1
    else
        return idx-1,idx
    end
end

"""
    resample array
"""
function resample_array(array::Matrix{Float64},old_t_vec,new_t_vec)
    new_array = Vector{Vector{Float64}}()
    for t in new_t_vec
        idx1,idx2 = get_interp_interval(old_t_vec,t)
        push!(new_array, linear_interp(array[idx1,:],array[idx2,:],(t-old_t_vec[idx1])/(old_t_vec[idx2]-old_t_vec[idx1])))
    end
    return collect(hcat(new_array...)')
end
function resample_array(array::A,old_t_vec,new_t_vec) where {A}
    new_array = Vector{Vector{Float64}}()
    for t in new_t_vec
        idx1,idx2 = get_interp_interval(old_t_vec,t)
        push!(new_array, linear_interp(array[idx1],array[idx2],(t-old_t_vec[idx1])/(old_t_vec[idx2]-old_t_vec[idx1])))
    end
    return new_array
end

export
    clip
"""
    clip(a,b,c)

Returns the closest element to `a` on the interval `[b,c]`
"""
clip(a,b,c) = max.(min.(a,c),b)

export
    draw_random_uniform

"""
    draw_random_elements(vec,n)

Draw `n` elements uniformly at random (without replacement) from vec.
"""
function draw_random_uniform(vec,n)
    N = length(vec)
    @assert N >= n "Cannot draw $n elements from vector of length $N"
    idxs = collect(1:N)
    selected_elements = Vector{eltype(vec)}()
    for i in 1:n
        k = rand(1:length(idxs))
        push!(selected_elements, vec[idxs[k]])
        deleteat!(idxs,k)
    end
    return selected_elements
end


# end # End of module GraphSorting
