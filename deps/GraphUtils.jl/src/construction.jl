# module Construction

# using LightGraphs, MetaGraphs
# using ..Arrays

export
    MatrixWrapper,
    IndicatorGrid,
    VtxGrid

abstract type MatrixWrapper <: AbstractMatrix{Int} end
for op in [:getindex,:setindex!,:get,:get!,:size,:length,:iterate]
    @eval Base.$op(m::MatrixWrapper,args...) = $op(m.mtx,args...)
end
Base.get(m::MatrixWrapper,idxs::NTuple{N,Int},args...) where {N} = get(m.mtx,idxs,args...)

"""
    IndicatorGrid

A matrix such that `M[i,j] == 0` indicates "free" and `M[i,j] == 1` indicates
"obstacle".
"""
struct IndicatorGrid{M<:AbstractMatrix} <: MatrixWrapper
    mtx::M
end
function Base.getindex(m::M,ind1::I,ind2::J) where {M<:IndicatorGrid,I<:AbstractRange{Int},J<:AbstractRange{Int}}
    M(getindex(m.mtx,ind1,ind2))
end

"""
    VtxGrid

A matrix such that `M[i,j] == v`, where v is the index of the
vertex whose coordinates are (i,j)
"""
struct VtxGrid{M<:AbstractMatrix} <: MatrixWrapper
    mtx::M
end

for t in [:IndicatorGrid,:VtxGrid]
    @eval $t(m::$t) = $t(m.mtx)
end

export
    initialize_vtx_grid_from_indicator_grid,
    initialize_regular_vtx_grid,
    initialize_dense_vtx_grid,
    construct_vtx_grid,
    initialize_grid_graph_from_vtx_grid,
    initialize_regular_grid_graph,
    initialize_grid_graph_with_obstacles

"""
    initialize_vtx_grid_from_indicator_grid()

Args:
    * grid: an indicator grid, where grid[i,j] == 0 denotes that cell (i,j) is
    free, and grid[i,j] != 0 denotes that the cell is impassable.

Returns:
    * A matrix whose non-zero elements represent the ids of vertices in a graph
    embedded in the 2D grid.
"""
function initialize_vtx_grid_from_indicator_grid(grid)
    K = VtxGrid(zeros(Int,size(grid)))
    idx = 1
    for i in 1:size(K,1)
        for j in 1:size(K,2)
            if grid[i,j] == 0
                K[i,j] = idx
                idx += 1
            end
        end
    end
    K
end
VtxGrid(m::IndicatorGrid) = initialize_vtx_grid_from_indicator_grid(m)

construct_indicator_grid_from_vtx_grid(vtx_grid) = IndicatorGrid(Int.(vtx_grid .== 0))
IndicatorGrid(m::VtxGrid) = construct_indicator_grid_from_vtx_grid(m)

export convolve_with_occupancy_kernel

"""
    convolve_with_occupancy_kernel(indicator_grid::IndicatorGrid,kernel)

Construct a new indicator grid corresponding to convolution of the original grid
with a kernel. The second argument may be a Matrix{Int} or a Tuple{Int,Int}
indicating the size for a ones() kernel.
"""
function convolve_with_occupancy_kernel(indicator_grid::IndicatorGrid,kernel::Matrix{Int})
    filtered_grid = IndicatorGrid(
        Int.(imfilter(indicator_grid, centered(kernel),Inner()) .> 0)
        )
end
function convolve_with_occupancy_kernel(indicator_grid::IndicatorGrid,shape::Tuple{Int,Int})
    convolve_with_occupancy_kernel(indicator_grid,ones(Int,shape))
end

export
    vtx_list_from_vtx_grid

function vtx_list_from_vtx_grid(m::VtxGrid)
    vtx_list = Vector{Tuple{Int,Int}}()
    for i in 1:size(m,1)
        for j in 1:size(m,2)
            if m[i,j] > 0
                push!(vtx_list,(i,j))
                @assert length(vtx_list) == m[i,j]
            end
        end
    end
    vtx_list
end

"""
    Returns a grid that represents a 2D environment with regularly spaced
    rectangular obstacles.

    Indicator grid values:
        0 == FREE
        1 == OCCUPIED
"""
function initialize_regular_indicator_grid(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [2;2],
    obs_offset = [1;1],
    env_pad = [1;1]
    )
    # generate occupancy grid representing the environment
    o = ones(Int,obs_width[1],obs_width[2]) # obstacle region
    op = pad_matrix(o,(obs_offset[1],obs_offset[2]),0) # padded obstacles region
    A = repeat(op,n_obstacles_x,n_obstacles_y)
    Ap = pad_matrix(A,(env_pad[1],env_pad[2]),0) # padded occupancy grid
    IndicatorGrid(Ap)
end
function initialize_regular_vtx_grid(;kwargs...)
    grid = initialize_regular_indicator_grid(;kwargs...)
    initialize_vtx_grid_from_indicator_grid(grid)
end

"""
    initialize_dense_vtx_grid
"""
initialize_dense_vtx_grid(x_dim::Int,y_dim::Int) = VtxGrid(collect(reshape(collect(1:x_dim*y_dim),y_dim,x_dim)'))

"""
    construct_vtx_grid

Returns a matrix `M` such that `MV[i,j] == v`, where v is the index of the
vertex whose coordinates are (i,j)

Arguments:
* vtxs : a list of integer coordinates
* dims : the dimensions of the grid
"""
function construct_vtx_grid(vtxs::V,dims) where {V<:Vector}
    vtx_grid = VtxGrid(zeros(Int,dims))
    for (k,vtx) in enumerate(vtxs)
        vtx_grid[vtx...] = k
    end
    vtx_grid
end


"""
    Returns a grid graph that represents a 2D environment with regularly spaced
    rectangular obstacles
"""
function initialize_grid_graph_from_vtx_grid(K::M) where {M<:Union{VtxGrid,Matrix{Int}}}
    G = MetaGraph()
    for i in 1:size(K,1)
        for j in 1:size(K,2)
            if K[i,j] != 0
                Graphs.add_vertex!(G)
                Graphs.add_edge!(G,nv(G),nv(G))
            end
        end
    end

    for i in 1:size(K,1)
        for j in 1:size(K,2)
            if K[i,j] != 0
                if j < size(K,2)
                    add_edge!(G,K[i,j],K[i,j+1],Dict(:direction=>:NORTH))
                end
                if j > 1
                    add_edge!(G,K[i,j],K[i,j-1],Dict(:direction=>:SOUTH))
                end
                if i < size(K,1)
                    add_edge!(G,K[i,j],K[i+1,j],Dict(:direction=>:EAST))
                end
                if i > 1
                    add_edge!(G,K[i,j],K[i-1,j],Dict(:direction=>:WEST))
                end
            end
        end
    end
    G
end

"""
    Returns a grid graph that represents a 2D environment with regularly spaced
    rectangular obstacles
"""
function initialize_regular_grid_graph(;
    env_offset = [1.0,1.0],
    env_scale = 2.0, # this is essentially the robot diameter
    kwargs...)

    K = initialize_regular_vtx_grid(;kwargs...)

    graph = initialize_grid_graph_from_vtx_grid(K)
    for i in 1:size(K,1)
        for j in 1:size(K,2)
            if K[i,j] != 0
                set_props!(graph, K[i,j],
                    Dict(:x=>env_offset[1] + env_scale*(i-1),
                    :y=>env_offset[2] + env_scale*(j-1))
                    )
            end
        end
    end
    return graph
end

"""
    Returns a grid graph that represents a 2D environment with obstacles placed
    over specific vertices
"""
function initialize_grid_graph_with_obstacles(
    dims,obstacles::Vector{Tuple{Int,Int}}=Vector{Tuple{Int,Int}}())
    indicator_grid = IndicatorGrid(zeros(Int,dims))
    for obs in obstacles
        indicator_grid[obs...] = 0
    end
    vtx_grid = VtxGrid(indicator_grid)
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    return G, vtx_grid
end

# end
