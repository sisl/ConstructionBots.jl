################################################################################
##############################    Tuple Stuff    ###############################
################################################################################
for op in [:typemin,:typemax]
    @eval Base.$op(::Type{Tuple{A,B,C,D,E,F}}) where {A,B,C,D,E,F}    = map(i->$op(i),(A,B,C,D,E,F))
    @eval Base.$op(::Type{Tuple{A,B,C,D,E}}) where {A,B,C,D,E}        = map(i->$op(i),(A,B,C,D,E))
    @eval Base.$op(::Type{Tuple{A,B,C,D}}) where {A,B,C,D}            = map(i->$op(i),(A,B,C,D))
    @eval Base.$op(::Type{Tuple{A,B,C}}) where {A,B,C}                = map(i->$op(i),(A,B,C))
    @eval Base.$op(::Type{Tuple{A,B}}) where {A,B}                    = map(i->$op(i),(A,B))
    @eval Base.$op(::Type{Tuple{A}}) where {A}                        = map(i->$op(i),(A,))
end
Base.Tuple{A,B,C,D,E,F}() where {A,B,C,D,E,F}    = map(i->i(0),(A,B,C,D,E,F))
Base.Tuple{A,B,C,D,E}() where {A,B,C,D,E}        = map(i->i(0),(A,B,C,D,E))
Base.Tuple{A,B,C,D}() where {A,B,C,D}            = map(i->i(0),(A,B,C,D))
Base.Tuple{A,B,C}() where {A,B,C}                = map(i->i(0),(A,B,C))
Base.Tuple{A,B}() where {A,B}                    = map(i->i(0),(A,B))
Base.Tuple{A}() where {A}                        = map(i->i(0),(A))

Base.typemin(::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemin(R),1:N))
Base.typemax(::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemax(R),1:N))
Base.NTuple{N,R}() where {N,R} = NTuple{N,R}(map(i->R(0), 1:N))
Base.NTuple{N,R}(val::R) where {N,R} = NTuple{N,R}(map(i->val, 1:N))
Base.typemin(c::Tuple) = typemin(typeof(c))
Base.typemax(c::Tuple) = typemax(typeof(c))

for op = (:(>), :(<), :(>=), :(<=))
    eval(quote
        Base.$op(a::T,b::R) where {T<:Tuple,R<:Real} = $op(a[1],b)
        Base.$op(b::R,a::T) where {T<:Tuple,R<:Real} = $op(b,a[1])
    end)
end

################################################################################
############################### Cost Model Stuff ###############################
################################################################################
export
    get_initial_cost,
    get_infeasible_cost,
    compute_path_cost,
    # get_transition_cost,
    accumulate_cost,
    add_heuristic_cost,
    aggregate_costs,

    AggregationFunction,
    MaxCost,
    SumCost,

    FullCostModel,
    get_aggregation_function,
    get_cost_model,
    cost_type,

    MetaCostModel,
    MetaCost,
    CompositeCostModel,
    construct_composite_cost_model,

    LowLevelCostModel,

    TravelTime,
    TravelDistance,
    FinalTime,
    NullCost,
    MakeSpan,
    SumOfTravelDistance,
    SumOfTravelTime

"""
    get_initial_cost(model)

Part of cost model interface. Defaults to zero.
"""
get_initial_cost(model)     = cost_type(model)(0)
# get_initial_cost(model::AbstractCostModel)     = cost_type(model)(0)

"""
    get_infeasible_cost(model)

Part of cost model interface. Defaults to zero.
"""
get_infeasible_cost(model)  = typemax(cost_type(model))
# get_infeasible_cost(model::AbstractCostModel)  = typemax(cost_type(model))

"""
    compute_path_cost(model,env,path,i)

Compute the cost of a path from scratch.
"""
compute_path_cost(model,env,path,i) = cost_type(model)(0)

# """
#     get_transition_cost(model,env,s,a,sp)
#     get_transition_cost(env,s,a,sp)
#     get_transition_cost(env,s,a)
#
# Part of cost model interface. Defines the cost of transitioning from state `s`
# to state `sp` via action `a` in environment `env` under cost model `model`.
# """
# function get_transition_cost end

"""
    accumulate_cost(model,current_cost,transition_cost)

Defines the way that a `transition_cost` updates the `current_cost` of a `Path`.
"""
function accumulate_cost end

"""
    add_heuristic_cost(cost_model,heuristic_model,cost,h_cost)

Defines the output heuristic cost that results from combining the current path
cost `cost` with a heuristic "cost-to-go" `h_cost`. Gener
"""
function add_heuristic_cost end
add_heuristic_cost(::C, cost, h_cost) where {C<:AbstractCostModel} = cost + h_cost
add_heuristic_cost(m::C, ::E, cost, h_cost) where {C<:AbstractCostModel,E<:AbstractLowLevelEnv} = add_heuristic_cost(m,cost,h_cost)
add_heuristic_cost(m::C, ::H, cost, h_cost) where {C<:AbstractCostModel,H<:AbstractCostModel}   = add_heuristic_cost(m,cost,h_cost)
add_heuristic_cost(m::C, h::H, ::E, cost, h_cost) where {C<:AbstractCostModel,H<:AbstractCostModel,E<:AbstractLowLevelEnv} = add_heuristic_cost(m,h,cost,h_cost)
add_heuristic_cost(env::E, cost, h_cost) where {E<:AbstractLowLevelEnv} = add_heuristic_cost(get_cost_model(env),get_heuristic_model(env),env,cost,h_cost)

"""
    compute_heuristic_cost(env,cost,sp)

Defaults to `add_heuristic_cost(env,cost,get_heuristic_cost(env,sp))`
Can be overridden so that state info can inform the heuristic cost directly.
"""
compute_heuristic_cost(m::C,h::H,env::E,cost,sp) where {C<:AbstractCostModel,H<:AbstractCostModel,E<:AbstractLowLevelEnv} = add_heuristic_cost(m,h,env,cost,get_heuristic_cost(h,env,sp))
compute_heuristic_cost(m::C,env::E,cost,sp) where {C<:AbstractCostModel,E<:AbstractLowLevelEnv} = add_heuristic_cost(m,env,cost,get_heuristic_cost(env,sp))
compute_heuristic_cost(env::E,args...) where {E<:AbstractLowLevelEnv} = compute_heuristic_cost(get_cost_model(env),get_heuristic_model(env),env,args...)

get_initial_cost(env::E) where {E<:AbstractLowLevelEnv}     = get_initial_cost(get_cost_model(env))
get_infeasible_cost(env::E) where {E<:AbstractLowLevelEnv}  = get_infeasible_cost(get_cost_model(env))
accumulate_cost(env::E, cost, transition_cost) where {E<:AbstractLowLevelEnv} = accumulate_cost(get_cost_model(env), cost, transition_cost)
# add_heuristic_cost(env::E, cost, h_cost) where {E<:AbstractLowLevelEnv} = add_heuristic_cost(env,get_cost_model(env),cost,h_cost)
get_transition_cost(env::E,s,a) where {E<:AbstractLowLevelEnv} = get_transition_cost(env,s,a,get_next_state(env,s,a))
get_transition_cost(env::E,s,a,sp) where {E<:AbstractLowLevelEnv} = get_transition_cost(get_cost_model(env),env,s,a,sp)

"""
    aggregate_costs(model, costs::Vector{T}) where {T}

Defines how costs from multiple distinct paths are combined into a single cost
for the whole solution. For example, the aggregation function for a
`SumOfTravelTime` objective is the `sum` of individual cost.
"""
function aggregate_costs end

"""
    a special version of aggregate_costs for the meta_env
"""
aggregate_costs_meta(m::AbstractCostModel,args...) = aggregate_costs(m,args...)

abstract type AggregationFunction end
struct MaxCost <: AggregationFunction end
(f::MaxCost)(costs...) = maximum(costs...)
struct SumCost <: AggregationFunction end
(f::SumCost)(costs...) = sum(costs...)

"""
    FullCostModel{F,T,M<:AbstractCostModel{T}} <: AbstractCostModel{T}

The `FullCostModel` defines all the behavior required for running CBS-based
algorithms.

Elements:
- f::F must be callable, and defines how a vector of path costs (e.g., the
    paths of a solution) should be combined into a single cost that reflects
    the cost of the entire path group together
- model::M<:AbstractCostModel{T} defines how the cost is computed during low
level search (when individual paths are being compared against each other).
"""
struct FullCostModel{F,T,M<:AbstractCostModel{T}} <: AbstractCostModel{T}
    f::F        # the aggregation function
    model::M    # the low level cost model
end
get_aggregation_function(m::FullCostModel)  = m.f
aggregate_costs(m::FullCostModel, costs)    = m.f(costs)
get_cost_model(m::FullCostModel)            = m.model
for op in [:accumulate_cost,:get_initial_cost,:compute_path_cost,
    :get_infeasible_cost,:add_heuristic_cost,:get_transition_cost]
    @eval $op(model::FullCostModel,args...) = $op(model.model,args...)
end
for T in [:FullCostModel]
    @eval begin
        add_heuristic_cost(m::$T, h::H, env::E, args...) where {H<:AbstractCostModel,E<:AbstractLowLevelEnv} = add_heuristic_cost(m.model, h, env, args...)
    end
end


"""
    CompositeCostModel{T}

Combines multiple cost models in a specific order (i.e., to use for cascaded
tie-breaking).
"""
struct CompositeCostModel{M<:Tuple,T<:Tuple} <: AbstractCostModel{T}
    cost_models::M
end
function construct_composite_cost_model(args...)
    models = Tuple(args)
    for m in models
        @assert typeof(m) <: AbstractCostModel
    end
    cost_types = map(m->cost_type(m),models)
    CompositeCostModel{typeof(models),Tuple{cost_types...}}(models)
end
# function get_transition_cost(model::C,env,s,a,sp) where {C<:CompositeCostModel}
#     cost_type(model)(map(m->get_transition_cost(m,env,s,a,sp), model.cost_models))
# end
function accumulate_cost(model::C, cost::T, transition_cost::T) where {T,M,C<:CompositeCostModel{M,T}}
    new_cost = map(x->accumulate_cost(x[1],x[2],x[3]),
    zip(model.cost_models, cost, transition_cost))
    T(new_cost)
end
function aggregate_costs(model::C, costs::Vector{T}) where {T,M,C<:CompositeCostModel{M,T}}
    aggregated_costs = map(
        i->aggregate_costs(model.cost_models[i], map(c->c[i], costs)), 1:length(model.cost_models))
    T(aggregated_costs)
end
function aggregate_costs_meta(model::C, costs::Vector{T}) where {T,M,C<:CompositeCostModel{M,T}}
    aggregated_costs = map(
        i->aggregate_costs_meta(model.cost_models[i], map(c->c[i], costs)), 1:length(model.cost_models))
    T(aggregated_costs)
end
for op in [:get_initial_cost,:get_infeasible_cost,:get_transition_cost,
    :compute_path_cost]
    @eval $op(model::CompositeCostModel,args...) = cost_type(model)(map(m->$op(m,args...),model.cost_models))
end
function add_heuristic_cost(model::C, cost::T, h_cost) where {T,M,C<:CompositeCostModel{M,T}}
    T(map(
        i->add_heuristic_cost(model.cost_models[i], cost[i], h_cost[i]),
        1:length(cost)
        ))
end
function compute_heuristic_cost(m::C,h::H,env::E,cost,s) where {C<:CompositeCostModel,H<:AbstractCostModel,E<:AbstractLowLevelEnv}
    # T([compute_heuristic_cost(env,m,c,s) for (m,c) in zip(model.cost_models,cost)])
    cost_type(m)(map(
        i->compute_heuristic_cost(
            m.cost_models[i],
            h.cost_models[i],
            env,
            cost[i],
            s),
        1:length(cost)
        ))
end

"""
    MetaCost

`MetaCost` maintaining separate costs for individual agents that have been
combined into a MetaAgent.
- independent_costs::Vector{T} a vector of costs, 1 per agent
- total_cost::T the total cost, which reflects the combined cost (its
    interpretation depends on the `MetaCostModel` used to define cost-
    upating behavior)
"""
struct MetaCost{T}
    independent_costs::Vector{T}
    total_cost::T
end
Base.isless(m1::MetaCost,m2::MetaCost) = m1.total_cost < m2.total_cost

"""
    MetaCostModel

Defines the cost-updating behavior of `MetaCost` for MetaAgent applications.
"""
struct MetaCostModel{T,M<:AbstractCostModel{T}} <: AbstractCostModel{MetaCost{T}}
    model::M
    num_agents::Int
end
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:MetaCostModel} = aggregate_costs(m.model, costs)
# function add_heuristic_cost(m::C, cost, h_cost) where {C<:MetaCostModel}
#     costs = map(i->add_heuristic_cost(
#         m.model,
#         cost.independent_costs[i],
#         h_cost[i]),1:m.num_agents)
#     MetaCost(costs, aggregate_costs_meta(m.model, costs))
# end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::Vector{T}) where {T,M<:MetaCostModel}
    new_costs = Vector{T}()
    for (i,(c1,c2)) in enumerate(zip(
        cost.independent_costs,
        transition_cost))
        push!(new_costs, accumulate_cost(model.model, c1, c2))
    end
    total_cost = aggregate_costs_meta(model.model,new_costs)
    new_cost = MetaCost{T}(new_costs,total_cost)
    return new_cost
end
function accumulate_cost(model::M, cost::MetaCost{T}, transition_cost::MetaCost{T}) where {T,M<:MetaCostModel}
    accumulate_cost(model, cost, transition_cost.independent_costs)
end
function get_initial_cost(model::C) where {C<:MetaCostModel}
    costs = map(a->get_initial_cost(model.model),1:model.num_agents)
    MetaCost(costs,aggregate_costs(model,costs))
end
function get_infeasible_cost(model::C) where {C<:MetaCostModel}
    costs = map(a->get_infeasible_cost(model.model),1:model.num_agents)
    MetaCost(costs,aggregate_costs(model,costs))
end

################################################################################
############################## Atomic Cost Models ##############################
################################################################################

"""
    LowLevelCostModel{C}

The low level cost model defines the objective to be optimized by the
solver at the low level. An optimal low level solver will return a path if a
feasible path exists) of minimal cost under the objective specified by the
associated LowLevelCostModel.
The parameter `C` defines the `cost_type` of the objective. The following
functions must be implemented for a `LowLevelCostModel` to be used:
* `get_initial_cost(model::LowLevelCostModel,env)` - returns
the initial_cost for a path
* `get_transition_cost(model::LowLevelCostModel{C},path::Path,s::S,a::A,
    sp::S) where {S,A,C}` - defines the cost associated with taking action
    `a` from state `s` to arrive in state `sp` according to the objective
    defined by `model` given that `s` is the "tip" of `path`.
* `accumulate_cost(model::LowLevelCostModel{C}, current_cost::C,
    transition_cost::C)` - defines how cost accumulates as new `PathNode`s
    are added to a Path.
"""
abstract type LowLevelCostModel{T} <: AbstractCostModel{T} end

export
    TransformCostModel

"""
    TransformCostModel{T,M<:LowLevelCostModel{T}} <: LowLevelCostModel{T}

Applies a transformation to an underlying cost model.
    e.g., `TransformCostModel(c->2*c, TravelTime())`
"""
struct TransformCostModel{T,M<:LowLevelCostModel{T}} <: LowLevelCostModel{T}
    f::Function
    model::M
end
# get_transition_cost(m::TransformCostModel,args...) = m.f(get_transition_cost(m.model,args...))
# accumulate_cost(m::TransformCostModel, args...)     = accumulate_cost(m.model,args...)
# get_initial_cost(m::TransformCostModel,args...)     = get_initial_cost(m.model,args...)
# get_infeasible_cost(m::TransformCostModel)          = get_infeasible_cost(m.model)
# add_heuristic_cost(m::TransformCostModel,args...)   = add_heuristic_cost(m.model,args...)
for op in [:accumulate_cost,:get_initial_cost,
    :get_infeasible_cost,:add_heuristic_cost,:get_transition_cost]
    @eval $op(model::TransformCostModel,args...) = model.f($op(model.model,args...))
end
# function compute_heuristic_cost(m::TransformCostModel,env::E,args...) where {E<:AbstractLowLevelEnv}
#     m.f(compute_heuristic_cost(m.model,env,args...))
# end
for op in [:add_heuristic_cost,:compute_heuristic_cost]
    @eval begin
        function $op(m::TransformCostModel, h::H, env::E, args...) where {H<:AbstractCostModel,E<:AbstractLowLevelEnv}
            m.f($op(m.model, h, env, args...))
        end
        function $op(m::TransformCostModel, env::E, args...) where {E<:AbstractLowLevelEnv}
            m.f($op(m.model, env, args...))
        end
    end
end
# add_heuristic_cost(m::TransformCostModel, h::H, env::E, args...) where {H<:AbstractCostModel,E<:AbstractLowLevelEnv} = m.f(add_heuristic_cost(m.model, h, env, args...))
# compute_heuristic_cost(m::TransformCostModel, h::H, env::E, args...) where {H<:AbstractCostModel,E<:AbstractLowLevelEnv} = m.f(add_heuristic_cost(m.model, h, env, args...))

"""
    TravelTime <: LowLevelCostModel{Float64}

Cost model that assigns cost equal to the duration of a path.
"""
struct TravelTime       <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelTime,      cost,transition_cost) = cost+transition_cost

"""
    TravelDistance <: LowLevelCostModel{Float64}

Cost model that assigns cost equal to the length (distance) of a path.
"""
struct TravelDistance   <: LowLevelCostModel{Float64} end
accumulate_cost(model::TravelDistance,  cost,transition_cost) = cost+transition_cost

struct FinalTime        <: LowLevelCostModel{Float64} end
accumulate_cost(model::FinalTime,       cost,transition_cost) = cost+transition_cost

"""
    NullCost <: LowLevelCostModel{Float64}

Cost equal to `0.0`.
"""
struct NullCost         <: LowLevelCostModel{Float64} end
get_transition_cost(model::F,env,s,a,sp) where {F<:NullCost} = 0.0
accumulate_cost(model::NullCost,        cost,transition_cost) = cost

export
    DeadlineCost,
    set_deadline!,
    FullDeadlineCost,
    # WeightedDeadlinesCost,
    MultiDeadlineCost,
    SumOfMakeSpans

abstract type AbstractDeadlineCost <: LowLevelCostModel{Float64} end
"""
    DeadlineCost

Identical to `TravelTime`, except for the behavior of
`add_heuristic_cost`.

add_heuristic_cost: `c = max(0.0, t + Δt - deadline)`
"""
mutable struct DeadlineCost     <: AbstractDeadlineCost
    deadline::Float64 # deadline
    m::TravelTime
end
DeadlineCost(deadline::R) where {R<:Real} = DeadlineCost(deadline,TravelTime())
function set_deadline!(m::DeadlineCost,t_max)
    m.deadline = minimum(t_max)
    return m
end
set_deadline!(m::C,args...) where {C<:AbstractCostModel} = nothing
set_deadline!(m::C,args...) where {C<:FullCostModel} = set_deadline!(m.model,args...)
set_deadline!(m::C,args...) where {C<:MetaCostModel} = set_deadline!(m.model,args...)
function set_deadline!(m::C,args...) where {C<:CompositeCostModel}
    for model in m.cost_models
        set_deadline!(model,args...)
    end
end
add_heuristic_cost(m::C, cost, h_cost) where {C<:DeadlineCost} = max(0.0, cost + h_cost - m.deadline) # assumes heuristic is PerfectHeuristic
for op in [:accumulate_cost,:get_initial_cost,:get_transition_cost,:compute_path_cost]
    @eval $op(model::AbstractDeadlineCost,args...) = $op(model.m,args...)
end
FullDeadlineCost(model::DeadlineCost) = FullCostModel(costs->max(0.0, maximum(costs)),model)

"""
    MultiDeadlineCost

Combines multiple deadlines according to some specified aggregation function.
"""
struct MultiDeadlineCost{F} <: AbstractDeadlineCost
    f::F # aggregation function
    tF::Vector{Float64}
    root_nodes::Vector{Int} # weights and deadlines correspond to root_nodes only
    weights::Vector{Float64}
    deadlines::Vector{Float64}
    m::TravelTime
end
const SumOfMakeSpans = MultiDeadlineCost{SumCost}
const MakeSpan = MultiDeadlineCost{MaxCost}
SumOfMakeSpans(tF,root_nodes,weights,deadlines) = MultiDeadlineCost(SumCost(),Float64.(tF),root_nodes,Float64.(weights),Float64.(deadlines),TravelTime())
SumOfMakeSpans() = MultiDeadlineCost(SumCost(),Float64[],Int[],Float64[],Float64[],TravelTime())
MakeSpan(tF,root_nodes,weights,deadlines) = MultiDeadlineCost(MaxCost(),Float64.(tF),root_nodes,Float64.(weights),Float64.(deadlines),TravelTime())
MakeSpan() = MultiDeadlineCost(MaxCost(),Float64[],Int[],Float64[],Float64[],TravelTime())
function set_deadline!(m::M,t_max) where {M<:MultiDeadlineCost}
    m.deadlines .= t_max
    return m
end
add_heuristic_cost(m::C, cost, h_cost) where {C<:MultiDeadlineCost} = m.f(m.weights .* max.(0.0, cost .+ h_cost .- m.deadlines)) # assumes heuristic is PerfectHeuristic
aggregate_costs(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost}  = m.f(m.tF[m.root_nodes] .* m.weights)
aggregate_costs_meta(m::C, costs::Vector{T}) where {T,C<:MultiDeadlineCost}  = maximum(costs)

# MakeSpan(model::FinalTime=FinalTime()) = FullCostModel(maximum,model)
SumOfTravelDistance(model::TravelDistance=TravelDistance()) = FullCostModel(sum,model)
SumOfTravelTime(model::TravelTime=TravelTime()) = FullCostModel(sum, model)

export EnvDeadlineCost
"""
    EnvDeadlineCost

Deadlines come from an external environment
"""
@with_kw struct EnvDeadlineCost{E,F} <: AbstractDeadlineCost
    env::E = nothing
    f::F   = SumCost()          # aggregation function
    m::TravelTime = TravelTime()
end
# EnvDeadlineCost() = EnvDeadlineCost(SumCost(),nothing,TravelTime())
# EnvDeadlineCost{E,F}() where {E,F} = EnvDeadlineCost(env=E(),f=F())
aggregate_costs(::EnvDeadlineCost,costs) = maximum(costs)

################################################################################
############################# HardConflictHeuristic ############################
################################################################################
export
    AbstractConflictTable,
    HardConflictTable,
    get_time_horizon,
    get_planned_vtx,
    reset_path!,
    set_path!,
    partially_set_path!,
    get_conflict_value,
    get_conflicting_paths,
    construct_empty_lookup_table

abstract type AbstractConflictTable end

"""
    HardConflictTable

Stores a lookup table of planned paths for all agents.
When agent `i` queries the table, `table.paths[i]` (the existing path plan
for agent `i`) must be subtracted so that the agent does not try to avoid
conflicts with itself.
"""
@with_kw struct HardConflictTable{V<:AbstractVector,M<:AbstractMatrix} <: AbstractConflictTable
    paths   ::Vector{V} = Vector{SparseVector{Int,Int}}()
    CAT     ::M         = SparseMatrixCSC(zeros(2,2)) # global table
    start_time::Int     = 0
end
get_time_horizon(h::T) where {T<:HardConflictTable} = size(h.CAT,2)
function get_planned_vtx(h::T,agent_idx::Int,t::Int) where {T<:HardConflictTable}
    t_idx = t + (1-h.start_time)
    if agent_idx == -1
        return 0
    else
        return get(h.paths[agent_idx], t_idx, -1)
    end
end
function reset_path!(h::T,path_idx::Int) where {V,M,T<:HardConflictTable{V,M}}
    # first remove the old path from the lookup table
    for (t,vtx) in enumerate(h.paths[path_idx])
        if vtx > 0
            h.CAT[vtx,t] = h.CAT[vtx,t] - 1
        end
    end
    # initialize an empty new path
    h.paths[path_idx] = V(zeros(Int,get_time_horizon(h)))
    return h
end
function set_path!(h::T,path_idx::Int,path::Vector{Int},start_time::Int=0) where {V,M,T<:HardConflictTable{V,M}}
    # println("Updating conflict table with path ", path, " for agent ",path_idx)
    reset_path!(h,path_idx)
    # now add new path vtxs to new path and lookup table
    for (i,vtx) in enumerate(path)
        t = (start_time-h.start_time) + i
        h.paths[path_idx][t] = vtx
        h.CAT[vtx,t] = h.CAT[vtx,t] + 1
    end
    h
end
"""
    partially_set_path!

Only replaces the cached path from start_time to length(path). Useful if you
want the remainder of the cached path to stay in the lookup table (i.e. for
repairing an existing plan).
"""
function partially_set_path!(h::T,path_idx::Int,path::Vector{Int},start_time::Int=0) where {V,M,T<:HardConflictTable{V,M}}
    # now add new path vtxs to new path and lookup table
    t0 = (start_time-h.start_time)
    for (i,vtx) in enumerate(path)
        t = t0 + i
        # try
        checkbounds(Bool,h.paths[path_idx],t) ? nothing : throw(SolverException("t = $(t) Out of bounds in partially_set_path! with start_time = $(h.start_time)"))
        old_vtx = h.paths[path_idx][t]
        if old_vtx != 0
            h.CAT[old_vtx,t] = h.CAT[old_vtx,t] - 1
        end
        h.paths[path_idx][t] = vtx
        h.CAT[vtx,t] = h.CAT[vtx,t] + 1
        # catch e
        #     throw(e)
        # end
    end
    h
end
function get_conflict_value(h::HardConflictTable,agent_idx::Int,vtx::Int,t::Int)
    t_idx = t + (1-h.start_time)
    try
        # c = h.CAT[vtx,t_idx]
        c = get(h.CAT, (vtx,t_idx), 0)
        if get_planned_vtx(h,agent_idx,t) == vtx # conflict with own trajectory
            c = c - 1
        end
        return c
    catch e
        @show vtx,t_idx,size(h.CAT)
        rethrow(e)
    end
end

"""
    get_conflicting_paths

Operates over a lookup table and returns a dictionary mapping path index to the
time index at which the conflict occurred
"""
function get_conflicting_paths(ct::T) where {T<:HardConflictTable}
    conflict_idxs = map(idx->Int.([idx.I...]), findall(ct.CAT .> 1))
    agent_id_to_time_idx = Dict{Int,Int}()
    for idx in conflict_idxs
        vtx = idx[1]
        t = idx[2]+ct.start_time-1
        for (agent_id,path) in enumerate(ct.paths)
            if get_planned_vtx(ct,agent_id,t) == vtx
                agent_id_to_time_idx[agent_id] = t
            end
        end
    end
    agent_id_to_time_idx
end

"""
    construct_empty_lookup_table(G,T::Int)

Returns an empty lookup table.
"""
construct_empty_lookup_table(V::Int,T::Int) = SparseMatrixCSC(zeros(V,T))
construct_empty_lookup_table(graph::G,T::Int) where {G<:AbstractGraph} = construct_empty_lookup_table(nv(graph),T)
function HardConflictTable(graph::G,T::Int,num_agents::Int) where {G<:AbstractGraph}
    HardConflictTable(
        paths = map(i->SparseVector(zeros(Int,T)),1:num_agents),
        CAT = construct_empty_lookup_table(graph,T)
        )
end
function HardConflictTable(graph::G,T::Float64,num_agents::Int) where {G<:AbstractGraph}
    @assert abs(T - Int(round(T))) < 0.01
    HardConflictTable(graph,Int(round(T)),num_agents)
end

################################################################################
############################### SoftConflictHeuristic ##############################
################################################################################
export
    SoftConflictTable,
    get_fat_path,
    add_fat_path_to_table!,
    populate_soft_lookup_table!

"""
    SoftConflictTable
"""
@with_kw struct SoftConflictTable{V,M<:AbstractMatrix} <: AbstractConflictTable
    paths   ::Vector{V} = Vector{SparseMatrixCSC{Float64,Int}}()
    CAT     ::M         = SparseMatrixCSC(zeros(2,2)) # global table
    start_time::Int     = 0
end
get_time_horizon(h::SoftConflictTable) = size(h.CAT,2)
get_conflict_value(h::SoftConflictTable,vtx::Int,t::Int) = h.CAT[vtx,t]
get_conflict_value(h::SoftConflictTable,agent_idx::Int,vtx::Int,t::Int) = h.CAT[vtx,t] - h.paths[agent_idx][vtx,t]

"""
    get_fat_path(G,D,start_vtx,goal_vtx)

returns a fat path through `G` from `start_vtx` to `goal_vtx`. Each set
of vertices in the fat path contains all vertices with distance d1 from
start_vtx and distance d2 to goal_vtx, where d1+d2 == the length of the
shortest path(s) from `start_vtx` to `goal_vtx`

G is a graph, D is the distance matrix
"""
function get_fat_path(G,D,start_vtx::Int,goal_vtx::Int)
    fat_path = Vector{Set{Int}}([Set{Int}(start_vtx)])
    for i in 1:D[start_vtx,goal_vtx]
        next_set = Set{Int}()
        for src_vtx in fat_path[end]
            for dst_vtx in outneighbors(G,src_vtx)
                if D[dst_vtx,goal_vtx] <= D[src_vtx,goal_vtx] - 1
                    push!(next_set, dst_vtx)
                end
            end
        end
        push!(fat_path, next_set)
    end
    fat_path
end

"""
    add_fat_path_to_table(CAT,fat_path)
"""
function add_fat_path_to_table!(CAT,fat_path,t0=0)
    for t in 1:length(fat_path)
        idxs = collect(fat_path[t])
        if t+t0 > 0
            CAT[idxs,t+t0] .+= 1.0/length(idxs)
        end
    end
end

"""
    populate_soft_lookup_table!(CAT,start_times,start_vtxs,goal_vtxs)
"""
function populate_soft_lookup_table!(CAT,G,D,start_vtxs,goal_vtxs,start_times=zeros(Int,length(start_vtxs)))
    for (s,t,g) in zip(start_vtxs,start_times,goal_vtxs)
        fat_path = get_fat_path(G,D,s,g)
        add_fat_path_to_table!(CAT,t0,fat_path)
    end
    CAT
end

"""
    construct_empty_lookup_table(graph,T::Int)

Returns a soft lookup table to encode possible paths for each agent through
`graph`. The argument `T` defines the time horizon of the lookup table.
"""
function SoftConflictTable(n_agents::Int,N::Int,T::Int,t0=0)
    SoftConflictTable(
        paths = map(i->spzeros(Float64,N,T),1:n_agents),
        CAT = spzeros(Float64,N,T),
        t0=t0
        )
end
function SoftConflictTable(n_agents::Int,N::Int,T::Float64,t0=0)
    @assert abs(T - Int(round(T))) < 0.01
    @assert abs(t0 - Int(round(t0))) < 0.01
    SoftConflictTable(n_agents,N,Int(round(T)),Int(round(t0)))
end

# """
#     `construct_empty_lookup_table(graph,T::Int)`
#
#     Returns a soft lookup table to encode possible paths for each agent through
#     `graph`. The argument `T` defines the time horizon of the lookup table.
# """
# function SoftConflictTable(graph,n_agents::Int,T::Int,t0=0)
#     SoftConflictTable(
#         paths = map(i->construct_empty_lookup_table(graph,T),1:n_agents),
#         CAT = construct_empty_lookup_table(graph,T),
#         t0=0
#         )
# end

# """
#     `construct_and_populate_soft_lookup_table!`
# """
# function SoftConflictTable(graph,
#         start_vtxs::Vector{Int},
#         goal_vtxs::Vector{Int},
#         start_times=zero(Int,length(start_vtxs),
#         T = Int(round(maximum(start_times) + nv(graph))),
#         )
#     )
#     CAT = construct_empty_lookup_table(graph,T)
#     D = get_dist_matrix(graph)
#     populate_soft_lookup_table!(CAT,graph,D,start_vtxs,goal_vtxs,start_times)
#     SoftConflictTable(CAT)
# end

################################################################################
############################# ConflictCostModel ############################
################################################################################
export
    ConflictCostModel,
    HardConflictCost,
    SoftConflictCost

struct ConflictCostModel{T<:Union{HardConflictTable,SoftConflictTable}} <: LowLevelCostModel{Float64}
    table::T
end
get_conflict_value(h::H, args...) where {H<:ConflictCostModel} = get_conflict_value(h.table, args...)
accumulate_cost(h::H,cost,transition_cost) where {H<:ConflictCostModel} = cost + transition_cost

HardConflictCost(args...) = FullCostModel(sum,ConflictCostModel(HardConflictTable(args...)))
SoftConflictCost(args...) = FullCostModel(sum,ConflictCostModel(SoftConflictTable(args...)))
SoftConflictCost(table::SoftConflictTable,args...) = FullCostModel(sum,ConflictCostModel(table))

get_time_horizon(h::H) where {H<:ConflictCostModel} = get_time_horizon(h.table)
get_planned_vtx(h::H,args...) where {H<:ConflictCostModel}  = get_planned_vtx(h.table,args...)
reset_path!(h::H,args...) where {H<:ConflictCostModel}    = reset_path!(h.table,args...)
reset_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
reset_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = reset_path!(h.model,args...)
function reset_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        reset_path!(m,args...)
    end
end
set_path!(h::H,args...) where {H<:ConflictCostModel}    = set_path!(h.table,args...)
set_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
set_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = set_path!(h.model,args...)
function set_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        set_path!(m,args...)
    end
end
partially_set_path!(h::H,args...) where {H<:ConflictCostModel}    = partially_set_path!(h.table,args...)
partially_set_path!(h::H,args...) where {H<:AbstractCostModel}    = nothing
partially_set_path!(h::FullCostModel{F,T,M},args...) where {F,T,M<:ConflictCostModel} = partially_set_path!(h.model,args...)
function partially_set_path!(h::H,args...) where {H<:CompositeCostModel}
    for m in h.cost_models
        partially_set_path!(m,args...)
    end
end
