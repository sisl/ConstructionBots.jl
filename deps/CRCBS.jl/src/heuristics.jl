export
    LowLevelSearchHeuristic,
    NullHeuristic,
    PerfectHeuristic,
    DefaultPerfectHeuristic,
    MultiStagePerfectHeuristic,
    ConflictTableHeuristic,
    HardConflictHeuristic,
    SoftConflictHeuristic,
    CompositeHeuristic,
        construct_composite_heuristic


"""
    LowLevelSearchHeuristic{C} <: AbstractCostModel{C}

Abstract type of a heuristic that returns cost of type `C`.
"""
abstract type LowLevelSearchHeuristic{C} <: AbstractCostModel{C} end
# function get_heuristic_cost(env::AbstractLowLevelEnv,m::AbstractCostModel,args...)
#     get_heuristic_cost(m,args...)
# end
################################################################################
############################## CompositeHeuristic ##############################
################################################################################
struct CompositeHeuristic{M<:Tuple,T<:Tuple} <: LowLevelSearchHeuristic{T}
    cost_models::M
end
function construct_composite_heuristic(args...)
    models = Tuple(args)
    for m in models
        @assert typeof(m) <: LowLevelSearchHeuristic
    end
    cost_types = map(m->cost_type(m),models)
    CompositeHeuristic{typeof(models),Tuple{cost_types...}}(models)
end
# cost_type(h::H) where {T,M,H<:CompositeHeuristic{T,M}} = T
function get_heuristic_cost(model::H,args...) where {T,M,H<:CompositeHeuristic{M,T}}
    T(map(h->get_heuristic_cost(h,args...), model.cost_models))
end
# function get_heuristic_cost(env,model::H,args...) where {T,M,H<:CompositeHeuristic{M,T}}
#     T(map(m->get_heuristic_cost(env,m,args...), model.cost_models))
# end

################################################################################
################################ NullHeuristic #################################
################################################################################
"""
    `NullHeuristic`
"""
struct NullHeuristic <: LowLevelSearchHeuristic{Float64} end
get_heuristic_cost(h::NullHeuristic,args...) = 0.0
get_heuristic_model(env) = NullHeuristic()
# get_heuristic_cost(h::NullHeuristic,env::AbstractLowLevelEnv,args...) = get_heuristic_cost(h,args...)

################################################################################
############################### PerfectHeuristic ###############################
################################################################################
"""
    `PerfectHeuristic`

    The Perfect Heuristic stores the exact distance between any vertex in a
    graph to all goal vertices specified during construction. The distances are
    stored in `dists`, a dictionary which maps a goal vertex `v` to a vector of
    distances from any other vertex in `1:nv(G)` to `v`.

    An example of how to access the distance from vertex `v1` to goal `v2`:
    `get_heuristic_cost(h,v1,v2) = h.dists[v2][v1]`
"""
@with_kw struct PerfectHeuristic{F} <: LowLevelSearchHeuristic{Float64}
    # dists::Dict{Int,Vector{Float64}} = Dict{Int,Vector{Float64}}()
    dists::F = (v1,v2)->0.0
end
function PerfectHeuristic(dists::Dict{Int,Vector{R}}) where {R<:Real}
    PerfectHeuristic((v1,v2)->get(get(dists,v1,Int[]),v2,0)*1.0)
end
get_heuristic_cost(h::PerfectHeuristic,goal_vtx::Int,vtx::Int) = h.dists(goal_vtx,vtx)
function PerfectHeuristic(graph,starts::Vector{Int},goals::Vector{Int})
    dists = Dict(v => gdistances(graph,v) for v in goals if 0 < v <= nv(graph))
    PerfectHeuristic(dists)
end
function PerfectHeuristic(dist_matrix::AbstractMatrix)
    dists = Dict(v => dist_matrix[:,v] for v in 1:size(dist_matrix,2))
    PerfectHeuristic(dists)
end

struct DefaultPerfectHeuristic <: LowLevelSearchHeuristic{Float64}
    h::PerfectHeuristic
end
get_heuristic_cost(h::DefaultPerfectHeuristic,goal_vtx::Int,vtx::Int) = haskey(h.h.dists,goal_vtx) ? get_heuristic_cost(h.h,goal_vtx,vtx) : 0.0

export EnvDistanceHeuristic
"""
    EnvDistanceHeuristic

A convenience struct that allows for the distance matrix to be stored in env
instead of the heuristic struct.
"""
struct EnvDistanceHeuristic <: LowLevelSearchHeuristic{Float64} end

"""
    MultiStagePerfectHeuristic

Stores multiple lookup tables corresponding to different stages of a Path-
Finding search. Each stage has a different goal. The heuristic value at a
particular stage must reflect not just the distance to the next goal but the
length of the path through all remaining goals.
"""
@with_kw struct MultiStagePerfectHeuristic <: LowLevelSearchHeuristic{Float64}
    dists::Dict{Int,Vector{Vector{Float64}}} = Dict{Int,Vector{Vector{Float64}}}()
end
get_heuristic_cost(h::MultiStagePerfectHeuristic,agent_idx::Int,stage::Int,vtx::Int) = h.dists[agent_idx][stage][vtx]
function construct_multi_stage_distance_array(G,goals)
    if length(goals) > 0
        dists = map(v->Float64.(gdistances(G,v)),goals)
        d = 0
        for i in reverse(1:length(goals)-1)
            d = d + dists[i][goals[i+1]]
            dists[i][:] = dists[i] .+ d
        end
        return dists
    end
    return Vector{Vector{Float64}}()
end
function MultiStagePerfectHeuristic(graph,goals::Vector{Vector{Int}})
    dists = Dict(idx => construct_multi_stage_distance_array(graph,g) for (idx,g) in enumerate(goals))
    MultiStagePerfectHeuristic(dists)
end

export MultiStageEnvDistanceHeuristic
"""
    MultiStageEnvDistanceHeuristic <: LowLevelSearchHeuristic{Float64}

`m.dists[i][stage]` stores the distance from that stage's goal to the final
stage's goal for agent `i`.
The heuristic cost is computed as follows:

`h = get_heuristic_cost(m.h,env,s) + cost_from_stage()`
"""
@with_kw struct MultiStageEnvDistanceHeuristic <: LowLevelSearchHeuristic{Float64}
    h::EnvDistanceHeuristic          = EnvDistanceHeuristic()
    dists::Dict{Int,Vector{Float64}} = Dict{Int,Vector{Float64}}()
end
function cost_from_stage(h::MultiStageEnvDistanceHeuristic,agent_idx::Int,stage::Int)
    h.dists[agent_idx][stage]
end
function construct_multi_stage_distance_list(env,goals)
    dists = Vector{Float64}()
    if length(goals) > 0
        d = 0
        g = goals[end]
        for v in reverse(goals)
            d = get_distance(env,v,g) + d
            push!(dists,d)
            g = v
        end
    end
    return reverse(dists)
end
function construct_multi_stage_env_distance_heuristic(env,goal_dict)
    MultiStageEnvDistanceHeuristic(
        EnvDistanceHeuristic(),
        Dict(idx => construct_multi_stage_distance_list(env,goals) for (idx,goals) in goal_dict)
    )
end


################################################################################
############################# ConflictTableHeuristic ###########################
################################################################################
"""
    ConflictTableHeuristic{T<:Union{HardConflictTable,SoftConflictTable}} <: LowLevelSearchHeuristic{Float64}

Heuristic model based on conflicts between paths.
"""
struct ConflictTableHeuristic{T<:Union{HardConflictTable,SoftConflictTable}} <: LowLevelSearchHeuristic{Float64}
    table::T
end
get_heuristic_cost(h::H,args...) where {H<:ConflictTableHeuristic} = get_conflict_value(h.table, args...)

HardConflictHeuristic(args...) = ConflictTableHeuristic(HardConflictTable(args...))
SoftConflictHeuristic(args...) = ConflictTableHeuristic(SoftConflictTable(args...))

get_time_horizon(h::H) where {H<:ConflictTableHeuristic} = get_time_horizon(h.table)
get_planned_vtx(h::H,args...) where {H<:ConflictTableHeuristic}  = get_planned_vtx(h.table,args...)
reset_path!(h::H,args...) where {T<:HardConflictTable,H<:ConflictTableHeuristic{T}}   = reset_path!(h.table,args...)
set_path!(h::H,args...) where {T<:HardConflictTable,H<:ConflictTableHeuristic{T}}     = set_path!(h.table,args...)
set_path!(h::H,args...) where {H<:LowLevelSearchHeuristic} = nothing
function set_path!(h::H,args...) where {H<:CompositeHeuristic}
    for m in h.cost_models
        set_path!(m,args...)
    end
end
