
export construct_vtx_map

"""
    construct_vtx_map(vtxs,dims)

Returns a matrix `M` such that `MV[i,j] == v`, where v is the index of the
vertex whose coordinates are (i,j)

Arguments:
* vtxs : a list of integer coordinates
* dims : the dimensions of the grid
"""
construct_vtx_map(vtxs, dims) = construct_vtx_grid(vtxs, dims).mtx

export construct_edge_cache

"""
    construct_edge_cache(vtxs,vtx_map)

Returns a `cache` such that `cache[v] = {(0,1),(0,0),...}`, the set of all
valid directions in which a robot may move from vertex `v`.

# Arguments:
- vtxs : a list of integer coordinates
- vtx_map : a matrix such that  `vtx_map[i,j] = v`, where  `vtxs[i,j] = v`.
"""
function construct_edge_cache(vtxs, vtx_map)
    edge_cache = Vector{Set{Tuple{Int,Int}}}()
    for (v, pt) in enumerate(vtxs)
        edge_list = Set{Tuple{Int,Int}}()
        for d in [(0, 0), (-1, 0), (0, 1), (1, 0), (0, -1)]
            if get(vtx_map, (pt[1] + d[1], pt[2] + d[2]), 0) >= 1
                push!(edge_list, d)
            end
        end
        push!(edge_cache, edge_list)
    end
    edge_cache
end

export validate_edge_cache

"""
    validate_edge_cache(G,vtxs,cache)

Verifies that for a graph `G` embedded in space such that vertex `v` has
coordinates `vtxs[v]`, every edge `v₁ → v₂` is stored in the edge cache. In
other words, for every edge `v₁ → v₂`, the unit direction vector obtained by
`vtxs[v₂] - vtxs[v₁]` is a member of `cache[v₁]`.

Arguments:
  - G : a graph on which the functions `vertices` and `outneighbors` can be called
  - vtxs : a list of integer coordinates
  - cache : an edge cache such that e.g. `cache[v] = {(0,1),(0,0),...}`, the set of all valid directions in which a robot may move from vertex `v`
"""
function validate_edge_cache(G, vtxs, cache)
    for v in vertices(G)
        vtx = vtxs[v]
        for v2 in outneighbors(G, v)
            vtx2 = vtxs[v2]
            d = [vtx2...] - [vtx...]
            d = tuple(d...)
            try
                @assert(d in cache[v], "$d not in $(edge_cache[v])")
            catch e
                print(e.msg)
                return false
            end
        end
    end
    return true
end

export construct_expanded_zones

"""
    construct_expanded_zones(vtxs,vtx_map,pickup_zones,dropoff_zones;
        shapes=[(1,1),(1,2),(2,1),(2,2)])

A utility for constructing drop-off/pick-up zones where large-footprint
objects need to be delivered. Returns a dictionary mapping vertices to a
dict of shape=>vtxs for expanded size delivery zones. For each starting
vertex, the expanded zone is selected as the appropriately sized region that
overlaps with the original vertex, does not overlap with any obstacles, and
has minimal overlap with all other vertices in the zone list.

Arguments:
  - vtxs : a list of integer coordinates
  - vtx_map : a matrix such that `vtx_map[i,j] == v`, where `vtxs[v] == (i,j)`
  - zones : a list of integer vertex indices identifying the zones to be
    expanded
"""
function construct_expanded_zones(
    vtxs,
    vtx_map,
    zones;
    shapes = [(1, 1), (1, 2), (2, 1), (2, 2)],
    minimize_overlap=true # if false, just go from top left
)
    expanded_zones = Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}(
        v => Dict{Tuple{Int,Int},Vector{Int}}() for v in zones)
    # populate heatmap with zones
    grid_map = IndicatorGrid(VtxGrid(vtx_map)) #Int.(vtx_map .== 0) # == 1 for all obstacles
    heatmap = zeros(size(grid_map))
    for v in zones
        heatmap[vtxs[v]...] += 1.0
    end
    for s in shapes
        # s = (2,2)
        filtered_grid = IndicatorGrid(Int.(imfilter(grid_map, centered(ones(s))) .> 0))
        for v in zones
            vtx = vtxs[v]
            best_cost = Inf
            best_vtx = (-1, -1)
            for i = 0:s[1]-1
                for j = 0:s[2]-1
                    # start_vtx = clip([vtx...]-[i,j],[1,1],[size(grid_map)...] .- s .+ 1)
                    idx1 = clip(vtx[1] - i, 1, size(grid_map, 1) - s[1] + 1)
                    idx2 = clip(vtx[2] - j, 1, size(grid_map, 2) - s[2] + 1)
                    if filtered_grid[idx1, idx2] == 0
                        if minimize_overlap
                            cost = sum(heatmap[idx1:idx1+s[1]-1, idx2:idx2+s[2]-1])
                            if cost < best_cost
                                best_cost = cost
                                best_vtx = (idx1, idx2)
                            end
                        else
                            if best_cost == Inf
                                best_cost = 0.0
                                best_vtx = (idx1, idx2)
                            end
                        end
                    end
                end
            end
            if any(best_vtx .== -1)
                println("Zone ",v," at ",vtx," unreachable for agents of shape ",s)
            else
                idx1, idx2 = best_vtx
                vtx_list = sort([vtx_map[idx1:idx1+s[1]-1, idx2:idx2+s[2]-1]...])
                expanded_zones[v][s] = vtx_list
            end
        end
    end
    expanded_zones
end
construct_expanded_zones(
    vtxs,
    vtx_map,
    pickup_zones,
    dropoff_zones;
    kwargs...,
) = construct_expanded_zones(
    vtxs,
    vtx_map,
    vcat(pickup_zones, dropoff_zones);
    kwargs...,
)

export validate_expanded_zones

"""
    validate_expanded_zones(vtx_map,expanded_zones)

Verify that all vertices of each expanded zone do not overlap with obstacle
regions, and that each expanded zone contains the original vertex.
"""
function validate_expanded_zones(vtx_map, expanded_zones)
    for (v, shape_dict) in expanded_zones
        for (s, vtx_list) in shape_dict
            try
                @assert(v in vtx_list, "$v not in expanded zone for itself!")
            catch e
                println(e.msg)
                return false
            end
            for vp in vtx_list
                try
                    @assert(
                        vp > 0,
                        "$vp in expanded zone of size $s for $v is out-of-bounds",
                    )
                catch e
                    println(e.msg)
                    return false
                end
            end
        end
    end
    return true
end

export
    SparseDistanceMatrix,
    remap_idx,
    get_val_and_status

"""
    SparseDistanceMatrix{G,M}

Stores a graph and sparse distance matrix. When the distance matrix is queried
for the distance between `start` and `goal`, it first checks that this distance
has been computed. If not, it will first compute the distances from all nodes to
`goal` before returning the queried distance.
"""
struct SparseDistanceMatrix{G,M}
    graph::G
    mtx::M
    SparseDistanceMatrix(g::G,m::S=spzeros(Int,nv(g),nv(g))) where {G,S} = new{G,S}(g,m)
end
Base.size(m::SparseDistanceMatrix) = size(m.mtx)
Base.valtype(m::SparseDistanceMatrix) = valtype(m.mtx)

function get_val_and_status(A::SparseMatrixCSC{T,M},i0,i1) where {T,M}
    if !(1 <= i0 <= size(A, 1) && 1 <= i1 <= size(A, 2)); throw(BoundsError()); end
    r1 = Int(SparseArrays.getcolptr(A)[i1])
    r2 = Int(SparseArrays.getcolptr(A)[i1+1]-1)
    if (r1 > r2); return zero(T), true; end
    r1 = searchsortedfirst(rowvals(A), i0, r1, r2, Base.Sort.Forward)
    empty_flag = ((r1 > r2) || (rowvals(A)[r1] != i0))
    val = empty_flag ? zero(T) : nonzeros(A)[r1]
    return val, empty_flag
end
function get_val_and_status(A::Matrix,i,j)
    if !(1 <= i0 <= size(A, 1) && 1 <= i1 <= size(A, 2)); throw(BoundsError()); end
    return A[i,j], true
end

function (m::SparseDistanceMatrix)(v1::Int,v2::Int)
    val, empty_flag = get_val_and_status(m.mtx,v1,v2)
    if empty_flag
        m.mtx[:,v2] .= gdistances(m.graph,v2;sort_alg=RadixSort)
        val = m.mtx[v1,v2]
    end
    return val
end
Base.getindex(m::SparseDistanceMatrix,v1::Int,v2::Int) = m(v1,v2)
Base.get(m::SparseDistanceMatrix,idxs::Tuple{Int,Int},args...) = m(idxs...)

export recompute_cached_distances!
"""
    recompute_cached_distances!

Recompute all cached distances (important if, e.g., the graph has been modified)
"""
function recompute_cached_distances!(m::SparseDistanceMatrix)
    _, cols, _ = findnz(m.mtx)
    for v in unique(cols)
        m.mtx[:,v] .= gdistances(m.graph,v;sort_alg=RadixSort)
    end
    return m
end

export remove_edges!
function remove_edges!(m::SparseDistanceMatrix,edge_set)
    for e in edge_set
        rem_edge!(m.graph,e)
    end
    recompute_cached_distances!(m)
end

export add_edges!
function add_edges!(m::SparseDistanceMatrix,edge_set)
    for e in edge_set
        add_edge!(m.graph,e)
    end
    recompute_cached_distances!(m)
end

export
    config_index_to_tuple,
    config_tuple_to_index

"""
    config_index_to_tuple(shape::Tuple{Int,Int}, idx::Int)

Convert a config index to a tuple coordinate representation. Assumes the
following layout for config indices given shape `(m,n)`:
          1          2         ... n
         ____________________________
    1   | 1          2         ... n
    2   | n+1        n+2       ... 2n
    ... |
    m   | (m-1)*n+1  (m-1)*n+2 ... m*n
"""
function config_index_to_tuple(shape::Tuple{Int,Int}, idx::Int)
    if !(1 <= idx <= prod(shape)); throw(BoundsError()); end
    (div(idx - 1, shape[2]) + 1, mod(idx - 1, shape[2]) + 1)
end
function config_tuple_to_index(shape::Tuple{Int,Int}, config::Tuple{Int,Int})
    if !all(1 .<= config .<= shape); throw(BoundsError()); end
    shape[2]*(config[1]-1) + config[2]
end

export
    RemappedDistanceMatrix

"""
    RemappedDistanceMatrix

Stores a distance matrix that corresponds to a "shifted" grid graph with a new
configuration space layered over a base graph. The shape `m.s` of
`m::RemappedDistanceMatrix` defines the convolution kernel used to generate the
modified.
"""
struct RemappedDistanceMatrix{M}
    mtx::M
    # relevant for remapping
    base_vtx_map::VtxGrid # VtxGrid for original graph
    base_vtxs::Vector{Tuple{Int,Int}} # VtxList for original graph
    vtx_map::VtxGrid
    vtxs::Vector{Tuple{Int,Int}}
    s::Tuple{Int,Int} # Shape of the large agent
    config::Tuple{Int,Int} # position of sub-agent relative to top left corner

    RemappedDistanceMatrix(m::M,base_map,base_vtxs,vtx_map,vtxs,
        s=(1,1),config=(1,1)) where {M} = new{M}(
        m,base_map,base_vtxs,vtx_map,vtxs,s,config
        )
end
function RemappedDistanceMatrix(m::RemappedDistanceMatrix,config::Tuple{Int,Int})
    RemappedDistanceMatrix(
        m.mtx,
        m.base_vtx_map,
        m.base_vtxs,
        m.vtx_map,
        m.vtxs,
        m.s,
        config
    )
end
RemappedDistanceMatrix(m::RemappedDistanceMatrix,config::Int) = RemappedDistanceMatrix(m,config_index_to_tuple(m.s,config))
function RemappedDistanceMatrix(grid::IndicatorGrid,shape::Tuple{Int,Int},config::Tuple{Int,Int}=(1,1))
    base_vtx_map = VtxGrid(grid)
    base_vtxs = vtx_list_from_vtx_grid(base_vtx_map)
    filtered_grid = convolve_with_occupancy_kernel(grid,shape)
    vtx_map = VtxGrid(filtered_grid)
    vtxs = vtx_list_from_vtx_grid(vtx_map)
    graph = initialize_grid_graph_from_vtx_grid(vtx_map)
    sm = SparseDistanceMatrix(graph)
    m = RemappedDistanceMatrix(sm,base_vtx_map,base_vtxs,vtx_map,vtxs,shape,config)
end

"""
    remap_idx(m::SparseDistanceMatrix,v,config)

Maps an index and configuration from the base grid to the transformed grid
represented by m.graph. `config` represents the position of the query pt
relative to the coordinates that correspond to `v`.
"""
function remap_idx(m::RemappedDistanceMatrix,v::Int,config::Tuple{Int,Int}=m.config)
    (1 <= v <= length(m.base_vtxs)) ? true : return -1
    vtx = m.base_vtxs[v]
    vtx_ = vtx .+ 1 .- config
    # @show config, vtx, vtx_, v
    # v_ = m.vtx_map[vtx_[1],vtx_[2]]
    v_ = get(m.vtx_map, vtx_, -1)
    # @show v_
    return v_
end
function remap_idx(m::RemappedDistanceMatrix,v::Int,config_idx::Int)
    remap_idx(m,v,config_index_to_tuple(m.s,config_idx))
end

function (m::RemappedDistanceMatrix)(v1::Int,v2::Int,config=m.config,
    default_val=valtype(m.mtx)(0)
    )
    i = remap_idx(m,v1,config)
    j = remap_idx(m,v2,config)
    all((1,1) .<= (i,j) .<= size(m.mtx)) ? true : return default_val
    return m.mtx[i,j]
end

function remap_edges(m::RemappedDistanceMatrix,edge_set)
    remapped_edge_set = Set{Edge}()
    for e in edge_set
        ep = Edge(remap_idx(m,e.src),remap_idx(m,e.dst))
        if ep.src != -1 && ep.dst != -1
            push!(remapped_edge_set,ep)
        end
    end
    remapped_edge_set
end
function remove_edges!(m::RemappedDistanceMatrix,edge_set)
    remove_edges!(m.mtx,remap_edges(m,edge_set))
    return m
end
function add_edges!(m::RemappedDistanceMatrix,edge_set)
    add_edges!(m.mtx,remap_edges(m,edge_set))
    return m
end

export
    DistMatrixMap,
    get_distance,
    get_team_config_dist_function

"""
    DistMatrixMap

Maps team size to the effective distance (computed by Djikstra) between
leader (top left) vtxs.
A DistMatrixMap is constructed by starting with a base environment grid
graph, which is represented as a binary occupancy grid. The occupancy grid
is then convolved with kernels of various sizes (which represent
configurations of robots moving as a team). The output of each convolution
represents a new occupancy grid corresponding to the workspace of the robot
team.
It is assumed that the "lead" robot is always in the top left of the
configuration. If a team of robots wishes to query the distance to a
particular target configuration, they pass the leader's current vtx,
the leader's target vtx, and the team configuration (shape) to the
DistMatrixMap, which returns the correct distance.
"""
struct DistMatrixMap{F}
    dist_mtxs::Dict{Tuple{Int,Int},RemappedDistanceMatrix{F}}
end
DistMatrixMap{F}() where {F} = DistMatrixMap{F}(Dict{Tuple{Int,Int},RemappedDistanceMatrix{F}}())
function get_team_config_dist_function(d::DistMatrixMap,shape,config_idx)
    mat = d.dist_mtxs[shape]
    return RemappedDistanceMatrix(mat,config_idx)
end
function add_edges!(m::DistMatrixMap,edge_set)
    for mat in values(m.dist_mtxs)
        add_edges!(mat,edge_set)
    end
end
function remove_edges!(m::DistMatrixMap,edge_set)
    for mat in values(m.dist_mtxs)
        remove_edges!(mat,edge_set)
    end
end

"""
    get_distance(mtx_map::DistMatrixMap,v1::Int,v2::Int,shape::Tuple{Int,Int}=(1,1),config_idx=1)

Returns the length of the minimum distance collision-free path between
vertices `v1` and `v2` for an object of footprint `shape`.

Arguments:
* mtx_map : a `DistMatrixMap`
* v1 : an integer corresponding to the source vertex in the graph
* v2 : an integer corresponding to the destination vertex in the graph
* shape : the footprint of the object that will move between `v1` and `v2`
* config_idx : an integer that identifies the position of the reference
    point within the footprint.
"""
function get_distance(
    mtx_map::DistMatrixMap,
    v1::Int,
    v2::Int,
    shape::Tuple{Int,Int} = (1, 1),
    config_idx = 1,
)
    D = mtx_map.dist_mtxs[shape](v1, v2, config_idx)
end
function get_distance(mtx::Matrix, v1::Int, v2::Int, args...)
    return get(mtx, (v1, v2), 0)
end
function DistMatrixMap(
        base_vtx_map::M,
        base_vtxs::Vector{Tuple{Int,Int}};
        shapes = [(1, 1), (1, 2), (2, 1), (2, 2)],
        ) where {M<:Union{VtxGrid,Matrix{Int}}}
    grid_map = IndicatorGrid(base_vtx_map)
    dist_mtx_map = DistMatrixMap(
        Dict(s=>RemappedDistanceMatrix(grid_map,s) for s in shapes)
    )
end
Base.getindex(d::DistMatrixMap, v1::Int, v2::Int) = get_distance(d, v1, v2, (1, 1), 1)
function (d::DistMatrixMap)(v1::Int,v2::Int,args...)
    get_distance(d,v1,v2,args...)
end



export
    GridFactoryEnvironment,
    # get_graph,
    get_x_dim,
    get_y_dim,
    get_cell_width,
    get_transition_time,
    get_vtxs,
    get_pickup_zones,
    get_dropoff_zones,
    get_obstacles,
    get_pickup_vtxs,
    get_dropoff_vtxs,
    get_obstacle_vtxs,
    get_num_free_vtxs,
    get_free_zones

abstract type AbstractFactoryEnv <:AbstractGraph{Int} end
"""
    GridFactoryEnvironment
"""
@with_kw struct GridFactoryEnvironment{G} <: AbstractFactoryEnv
    graph::G = MetaDiGraph()
    x_dim::Int = 20
    y_dim::Int = 20
    cell_width::Float64 = 0.5
    transition_time::Float64 = 2.0
    vtxs::Vector{Tuple{Int,Int}} = Vector{Tuple{Int,Int}}()
    pickup_zones::Vector{Int}   = collect(1:length(vtxs))
    dropoff_zones::Vector{Int}  = collect(1:length(vtxs))
    free_zones::Vector{Int}     = collect(1:length(vtxs))
    obstacles::Vector{Tuple{Int,Int}} = Vector{Tuple{Int,Int}}()
    vtx_map::VtxGrid = construct_vtx_grid(vtxs, (x_dim, y_dim))
    edge_cache::Vector{Set{Tuple{Int,Int}}} = construct_edge_cache(vtxs,vtx_map)
    expanded_zones::Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}} = construct_expanded_zones(
        vtxs, vtx_map, pickup_zones, dropoff_zones;
        shapes = filter(t->all(t.<=(x_dim,y_dim)),[(1,1),(1,2),(2,1),(2,2)])
        )
    dist_function::DistMatrixMap = DistMatrixMap(vtx_map, vtxs)
end
# get_graph(env::GridFactoryEnvironment) = env.graph
get_x_dim(env::GridFactoryEnvironment) = env.x_dim
get_y_dim(env::GridFactoryEnvironment) = env.y_dim
get_cell_width(env::GridFactoryEnvironment) = env.cell_width
get_transition_time(env::GridFactoryEnvironment) = env.transition_time
get_vtxs(env::GridFactoryEnvironment) = env.vtxs
get_pickup_zones(env::GridFactoryEnvironment) = env.pickup_zones
get_dropoff_zones(env::GridFactoryEnvironment) = env.dropoff_zones
get_obstacles(env::GridFactoryEnvironment) = env.obstacles
get_pickup_vtxs(env::GridFactoryEnvironment) = get_vtxs(env)[get_pickup_zones(env)]
get_dropoff_vtxs(env::GridFactoryEnvironment) =get_vtxs(env)[get_dropoff_zones(env)]
get_obstacle_vtxs(env::GridFactoryEnvironment) = get_obstacles(env)
get_dist_matrix(env::GridFactoryEnvironment) = env.dist_function
function GridFactoryEnvironment(
    env::E,
    graph::G,
) where {E<:GridFactoryEnvironment,G<:AbstractGraph}
    GridFactoryEnvironment(
        graph = graph,
        x_dim = env.x_dim,
        y_dim = env.y_dim,
        cell_width = env.cell_width,
        transition_time = env.transition_time,
        vtxs = env.vtxs,
        pickup_zones = env.pickup_zones,
        dropoff_zones = env.dropoff_zones,
        obstacles = env.obstacles,
        vtx_map = env.vtx_map,
    )
end

Base.zero(env::GridFactoryEnvironment{G}) where {G} =
    GridFactoryEnvironment(env, graph = G())
Graphs.edges(env::GridFactoryEnvironment) = edges(env.graph)
for op in [:edgetype,:edges,:has_edge,:has_vertex,:inneighbors,:is_directed,
    :ne,:nv,:outneighbors,:vertices]
    @eval Graphs.$op(env::GridFactoryEnvironment, args...) = $op(env.graph,args...)
end
function vtx_to_idx(env::GridFactoryEnvironment,vtx::Tuple{Int,Int})
    env.vtx_map[vtx[1],vtx[2]]
end
function idx_to_vtx(env::GridFactoryEnvironment,idx::Int)
    env.vtxs[idx]
end
function idx_from_offset(env::GridFactoryEnvironment,idx::Int,offset::Tuple{Int,Int}=(0,0))
    vtx_to_idx(env,idx_to_vtx(env,idx) .+ offset)
end
# get_x(env::E,v::Int) where {E<:GridFactoryEnvironment} = get_vtxs(env)[v][1]
# get_y(env::E,v::Int) where {E<:GridFactoryEnvironment} = get_vtxs(env)[v][2]
# get_θ(env::E,v::Int) where {E<:GridFactoryEnvironment} = 0.0
get_num_free_vtxs(env::GridFactoryEnvironment) = length(env.free_zones)
get_free_zones(env::GridFactoryEnvironment) = env.free_zones
get_distance(env::GridFactoryEnvironment,args...) = get_distance(env.dist_function,args...)
get_team_config_dist_function(env::GridFactoryEnvironment,args...) = get_team_config_dist_function(env.dist_function,args...)
function remove_edges!(env::GridFactoryEnvironment,edge_set)
    for e in edge_set
        rem_edge!(env.graph,e)
    end
    remove_edges!(env.dist_function,edge_set)
    return env
end
function add_edges!(env::GridFactoryEnvironment,edge_set)
    for e in edge_set
        add_edge!(env.graph,e)
    end
    add_edges!(env.dist_function,edge_set)
    return env
end

################################################################################
################################ READ AND WRITE ################################
################################################################################
export read_env

function TOML.parse(env::GridFactoryEnvironment)
    toml_dict = Dict()
    toml_dict["title"] = "GridFactoryEnvironment"
    toml_dict["x_dim"] = get_x_dim(env)
    toml_dict["y_dim"] = get_y_dim(env)
    toml_dict["cell_width"] = get_cell_width(env)
    toml_dict["transition_time"] = get_transition_time(env)
    toml_dict["vtxs"] = map(tup -> [tup[1], tup[2]], get_vtxs(env))
    toml_dict["pickup_zones"] = get_pickup_zones(env)
    toml_dict["dropoff_zones"] = get_dropoff_zones(env)
    toml_dict["obstacles"] = map(tup -> [tup[1], tup[2]], get_obstacles(env))
    return toml_dict
end
function TOML.print(io, env::GridFactoryEnvironment)
    TOML.print(io, TOML.parse(env))
end

"""
    read_env(io)

Loads saved environment from a .toml file.
"""
function read_env(io)
    toml_dict = TOML.parsefile(io)
    x_dim = toml_dict["x_dim"]
    y_dim = toml_dict["y_dim"]
    vtxs = map(arr -> (arr[1], arr[2]), toml_dict["vtxs"])
    vtx_grid = construct_vtx_grid(vtxs,(x_dim, y_dim))
    graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    env = GridFactoryEnvironment(
        graph = graph,
        x_dim = toml_dict["x_dim"],
        y_dim = toml_dict["y_dim"],
        cell_width = toml_dict["cell_width"],
        transition_time = toml_dict["transition_time"],
        vtxs = map(arr -> (arr[1], arr[2]), toml_dict["vtxs"]),
        pickup_zones = toml_dict["pickup_zones"],
        dropoff_zones = toml_dict["dropoff_zones"],
        obstacles = map(arr -> (arr[1], arr[2]), toml_dict["obstacles"]),
    )
    # GridFactoryEnvironment(env, initialize_factory_graph(env))
end

################################################################################
################################ INITIALIZATION ################################
################################################################################

export construct_regular_factory_world

"""
    construct_regular_factory_world()

Returns a `GridFactoryEnvironment` with regularly spaced obstacle regions
surrounded by alternating pick-up and drop-off locations.

Keyword Arguments:
* n_obstacles_x = 2 : number of obstacles in x direction
* n_obstacles_y = 2 : number of obstacles in y direction
* obs_width = [2;2] : obstacle width in both directions
* obs_offset = [1;1] : width of buffer region around each obstacle
* env_pad = [1;1] : env pad
* env_scale = 0.5 : determines the width of each grid cell when the
    coordinates of the environment are transformed to continuous Cartesian
    space.
* transition_time = 2.0 : determines the nominal travel time for a robot
    to move from one grid cell to an adjacent one.
"""
function construct_regular_factory_world(
        ;
        n_obstacles_x = 2,
        n_obstacles_y = 2,
        obs_width = [2; 2],
        obs_offset = [1; 1],
        env_pad = [1; 1],
        env_scale = 0.5,
        transition_time = 2.0,
        exclude_from_free = false,
    )
    PICKUP_FLAG = -1
    DROPOFF_FLAG = -2
    OBSTACLE_FLAG = 0
    NORMAL_VTX_FLAG = 1
    # generate occupancy grid representing the environment
    o = OBSTACLE_FLAG * ones(Int, obs_width[1], obs_width[2]) # obstacle region
    op = pad_matrix(o, (1, 1), NORMAL_VTX_FLAG) # padded obstacles region
    flag = PICKUP_FLAG
    for i = 1:size(o, 1)
        op[i+1, 1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for j = 1:size(o, 2)
        op[end, j+1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for i in reverse(collect(1:size(o, 1)))
        op[i+1, end] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for j in reverse(collect(1:size(o, 2)))
        op[1, j+1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    op = pad_matrix(op, (obs_offset[1] - 1, obs_offset[2] - 1), NORMAL_VTX_FLAG) # padded obstacles region
    A = repeat(op, n_obstacles_x, n_obstacles_y)
    Ap = pad_matrix(A, (env_pad[1], env_pad[2]), NORMAL_VTX_FLAG) # padded occupancy grid
    K = zeros(Int, size(Ap))

    pickup_zones = Vector{Int}()
    dropoff_zones = Vector{Int}()
    obstacles = Vector{Tuple{Int,Int}}()
    vtxs = Vector{Tuple{Int,Int}}()
    k = 0
    for i = 1:size(Ap, 1)
        for j = 1:size(Ap, 2)
            if Ap[i, j] == OBSTACLE_FLAG
                push!(obstacles, (i, j))
            else
                k += 1
                K[i, j] = k
                push!(vtxs, (i, j))
                if Ap[i, j] == PICKUP_FLAG
                    push!(pickup_zones, k)
                elseif Ap[i, j] == DROPOFF_FLAG
                    push!(dropoff_zones, k)
                end
            end
        end
    end
    if exclude_from_free
        free_zones = setdiff(collect(1:length(vtxs)),union(pickup_zones,dropoff_zones))
    else
        free_zones = collect(1:length(vtxs))
    end

    graph = initialize_grid_graph_from_vtx_grid(K)

    env = GridFactoryEnvironment(
        graph = graph,
        x_dim = size(Ap, 1),
        y_dim = size(Ap, 2),
        cell_width = env_scale,
        transition_time = transition_time,
        vtxs = vtxs,
        pickup_zones = pickup_zones,
        dropoff_zones = dropoff_zones,
        free_zones = free_zones,
        obstacles = obstacles,
    )

    return env
end

export construct_factory_env_from_vtx_grid

"""
    construct_factory_env_from_vtx_grid(vtx_grid;kwargs...)

# Arguments:
  - vtx_grid : a matrix such that `vtx_grid[i,j] > 0` represents free space,
    otherwise an obstacle.

Keyword Arguments:
* cell_width = 0.5 : determines the width of each grid cell when the
    coordinates of the environment are transformed to continuous Cartesian
    space.
* transition_time = 2.0 : determines the nominal travel time for a robot
    to move from one grid cell to an adjacent one.
* pickup_zones = Int[] : a list of vertices that represent pick-up points
* dropoff_zones = Int[] : a list of vertices that represent drop-off points
"""
function construct_factory_env_from_vtx_grid(
        vtx_grid;
        cell_width = 1.0,
        transition_time = 1.0,
        pickup_zones = collect(1:sum(vtx_grid .> 0)),
        dropoff_zones = collect(1:sum(vtx_grid .> 0)),
    )

    graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    vtxs = Vector{Tuple{Int,Int}}()
    for i = 1:size(vtx_grid, 1)
        for j = 1:size(vtx_grid, 2)
            if vtx_grid[i, j] > 0
                push!(vtxs, (i, j))
            end
        end
    end

    env = GridFactoryEnvironment(
        graph = graph,
        x_dim = size(vtx_grid, 1),
        y_dim = size(vtx_grid, 2),
        cell_width = cell_width,
        transition_time = transition_time,
        vtxs = vtxs,
        pickup_zones = pickup_zones,
        dropoff_zones = dropoff_zones,
    )
end

export
    construct_factory_env_from_indicator_grid

"""
    construct_factory_env_from_indicator_grid

Args:
    * grid: an indicator grid whose zero entries denote free space (non-zero
    denotes obstacle)
"""
function construct_factory_env_from_indicator_grid(grid;kwargs...)
    construct_factory_env_from_vtx_grid(
        initialize_vtx_grid_from_indicator_grid(grid);kwargs...
    )
end
