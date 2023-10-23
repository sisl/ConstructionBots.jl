export
    MetaAgentCBS

# CBS submodule
module MetaAgentCBS

using ..CRCBS
using Parameters, Graphs, DataStructures


@with_kw struct State{S}
    states::Vector{S} = Vector{S}()
end
State(states::T) where {T<:Tuple} = State([states...])
Base.string(s::State) = string("(",prod(map(s->"$(string(s)),",s.states)),")")
@with_kw struct Action{A}
    actions::Vector{A} = Vector{A}()
end
Action(actions::T) where {T<:Tuple} = Action([actions...])
Base.hash(s::State{S}) where {S} = hash(s.states)
Base.:(==)(s1::S,s2::S) where {S<:State} = hash(s1) == hash(s2)
Base.string(a::Action) = string("(",prod(map(a->"$(string(a)),",a.actions)),")")

get_start_index(path::Path{MS,A,C}) where {S<:AbstractGraphState,MS<:State{S},A,C} = get_t(get_initial_state(path).states[1])

abstract type AbstractMetaEnv{S,A,T,C} <: AbstractLowLevelEnv{State{S},Action{A},MetaCostModel{T,C}} end
@with_kw struct LowLevelEnv{S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}} <: AbstractMetaEnv{S,A,T,C}
    envs::Vector{E}             = Vector{E}()
    agent_idxs::Vector{Int}     = Vector{Int}()
    cost_model::MetaCostModel{T,C} = MetaCostModel(
        FullCostModel(sum,TravelTime()),length(envs))
end
# CRCBS.state_type(env::AbstractMetaEnv{S,A,T,C}) where {S,A,T,C} = State{S}
function CRCBS.compute_heuristic_cost(env::MetaAgentCBS.AbstractMetaEnv,cost,sp)
# function compute_heuristic_cost(m::MetaCostModel,::AbstractCostModel,env::E,cost,sp) where {E<:AbstractLowLevelEnv}
    m = get_cost_model(env)
    # h_cost = get_heuristic_cost(env,sp)
    costs = map(i->CRCBS.compute_heuristic_cost(
        get_envs(env)[i],
        cost.independent_costs[i],
        sp.states[i],
        # h_cost[i],
        ),1:m.num_agents)
    CRCBS.MetaCost(costs, CRCBS.aggregate_costs_meta(m.model, costs))
end

"""
    TeamMetaEnv{S,A,T,C,E}

Represents an env where a team of agents must move in rigid formation. The
function `CRCBS.get_possible_actions` must be overridden for all environments to
reflect this rigidity.
"""
@with_kw struct TeamMetaEnv{S,A,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}} <: AbstractMetaEnv{S,A,T,C}
    envs::Vector{E}             = Vector{E}()
    agent_idxs::Vector{Int}     = Vector{Int}()
    cost_model::MetaCostModel{T,C} = MetaCostModel(
        FullCostModel(sum,TravelTime()),length(envs))
end
get_envs(env::AbstractMetaEnv)                 = env.envs
get_agent_idxs(env::AbstractMetaEnv)           = env.agent_idxs
CRCBS.get_cost_model(env::AbstractMetaEnv)     = env.cost_model

function construct_meta_env(envs::Vector{E},idxs::Vector{Int}) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    LowLevelEnv{S,A,T,C,E}(envs=envs,agent_idxs=idxs)
end
function construct_meta_env(envs::Vector{E},idxs::Vector{Int},c_model::C) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    model = MetaCostModel(c_model,length(envs))
    LowLevelEnv{S,A,T,C,E}(envs=envs,agent_idxs=idxs,cost_model=model)
end
function construct_team_env(envs::Vector{E},idxs::Vector{Int}) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    TeamMetaEnv{S,A,T,C,E}(envs=envs,agent_idxs=idxs)
end
function construct_team_env(envs::Vector{E},idxs::Vector{Int},c_model::C) where {S,A,T,C<:AbstractCostModel{T},E <: AbstractLowLevelEnv{S,A,C}}
    model = MetaCostModel(c_model,length(envs))
    TeamMetaEnv{S,A,T,C,E}(envs=envs,agent_idxs=idxs,cost_model=model)
end

function split_meta_path_node(path_node::PathNode{State{S},Action{A}}) where {S,A}
    return map(args->PathNode(args...), zip(
        get_s(path_node).states,
        get_a(path_node).actions,
        get_sp(path_node).states
        ))
end
function construct_meta_path_node(nodes::Vector{N}) where {N<:PathNode}
    PathNode(
        State(map(get_s,nodes)),
        Action(map(get_a,nodes)),
        State(map(get_sp,nodes)),
    )
end
construct_meta_path_node(nodes::Tuple) = construct_meta_path_node([nodes...])

# # TODO implement MetaPath
# @with_kw struct MetaPath{S,A,C} <: AbstractPath
#     paths       ::Vector{Path{S,A,C}}   = Vector{Path{S,A,C}}()
#     s0          ::State                 = get_s(get(path_nodes, 1, PathNode{S,A}()))
#     cost        ::MetaCost{C}           = MetaCost(map(get_cost,paths),typemax(C))
# end
# state_type(p::MetaPath{S,A,C}) where {S,A,C}    = State{S}
# action_type(p::MetaPath{S,A,C}) where {S,A,C}   = Action{A}
# cost_type(p::MetaPath{S,A,C}) where {S,A,C}     = MetaCost{C}
# Base.get(p::MetaPath,i,default=node_type(p)()) = construct_meta_path_node(map(p->get(p,i),p.paths))
# Base.getindex(p::MetaPath,i)    = construct_meta_path_node(map(p->getindex(p,i),p.paths))
# function Base.cat(p::P,x::N,args...) where {P<:MetaPath,N<:PathNode}
#     new_paths = typeof(p.paths)()
#     for (path,node) in zip(p.paths,split_path_node(x))
#         setindex!(path,node,i)
#         push!(new_paths,cat(path,node,args...))
#     end
#     P(s0=p.s0,paths=new_paths,cost=p.cost)
# end
# function Base.setindex!(p::P,x,i) where {P<:MetaPath}
#     for (path,node) in zip(p.paths,split_path_node(x))
#         setindex!(path,node,i)
#     end
# end
# Base.length(p::MetaPath) = maximum(map(length,p.paths))
# function Base.push!(p::MetaPath{S,A,C},n::PathNode{State{S},Action{A}}) where {S,A,C}
#     for (path,node) in zip(p.paths,split_path_node(n))
#         push!(path,node)
#     end
#     p
# end
# get_cost(p::MetaPath) = p.cost
# function set_cost!(p::MetaPath,meta_cost::MetaCost)
#     for (path,cost) in zip(p.paths,meta_cost.independent_costs)
#         set_cost!(path)
#     end
#     p.cost = meta_cost.cost
# end
# Base.copy(p::MetaPath) = MetaPath(s0=p.s0,paths=map(copy,p.paths),cost=p.cost)
# get_initial_state(path::MetaPath) = State(map(get_initial_state,path.paths))
# get_final_state(path::MetaPath) = State(map(get_final_state,path.paths))

function construct_meta_path(paths::Vector{P},cost) where {S,A,C,P<:Path{S,A,C}}
    @assert all(map(length,paths) .- maximum(map(length,paths)) .== 0) "$(map(length,paths))"
    starts = map(get_initial_state,paths)
    costs = map(get_cost, paths)
    path_nodes = Vector{PathNode{State{S},Action{A}}}()
    for nodes in zip(map(p->p.path_nodes,paths)...)
        push!(path_nodes, construct_meta_path_node(nodes))
    end
    meta_path = Path(
        path_nodes = path_nodes,
        s0=State(starts),
        cost = MetaCost(costs,cost)
    )
end
function CRCBS.build_env(solver::MetaAgentCBS_Solver, mapf, node, i)
    group = node.solution.group_idxs[i]
    construct_meta_env(
        [build_env(mapf, node, j) for j in group],
        group,
        get_cost_model(mapf.env)
        )
end
CRCBS.get_start(mapf::MAPF,env::AbstractMetaEnv,i) = State([get_start(mapf,e,j) for (e,j) in zip(get_envs(env), get_agent_idxs(env))])
################################################################################
################################################################################
################################################################################

"""
    `split_path(path::Path{State{S},Action{A}}) where {S,A}`

    Helper for breaking a meta-agent path into a set of single-agent paths
"""
function split_path(path::Path{State{S},Action{A},MetaCost{C}}) where {S,A,C}
    N = length(get_s(get_path_node(path, 1)).states)
    paths = [Path{S,A,C}(s0=path.s0.states[i],cost=get_cost(path).independent_costs[i]) for i in 1:N]
    for t in 1:length(path)
        path_node = get_path_node(path, t)
        for (p,n) in zip(paths,split_meta_path_node(path_node))
            push!(p,n)
        end
    end
    return paths
end
function CRCBS.set_solution_path!(solution::MetaSolution, meta_path, group_idx)
    for (idx,path) in zip(solution.group_idxs[group_idx],split_path(meta_path))
        set_solution_path!(solution.solution, path, idx)
    end
end
function CRCBS.set_path_cost!(solution::MetaSolution, meta_cost, group_idx)
    for (idx,cost) in zip(solution.group_idxs[group_idx],meta_cost.independent_costs)
        set_path_cost!(solution.solution, cost, idx)
    end
end
function CRCBS.get_heuristic_cost(env::E,state::State) where {E<:AbstractMetaEnv}
    c = [get_heuristic_cost(e,s) for (e,s) in zip(get_envs(env), state.states)]
end
function CRCBS.states_match(s1::State,s2::State)
    for (state1, state2) in zip(s1.states,s2.states)
        if !states_match(state1,state2)
            return false
        end
    end
    return true
end
function CRCBS.is_goal(env::E,state::State) where {E<:AbstractMetaEnv}
    for (e,s) in zip(get_envs(env), state.states)
        if !is_goal(e,s)
            return false
        end
    end
    return true
end
CRCBS.wait(s::State) = Action(map(CRCBS.wait, s.states))
CRCBS.wait(env::AbstractMetaEnv,s::State) = Action(map(args->CRCBS.wait(args...),zip(get_envs(env),s.states)))
# get_possible_actions
struct ActionIter{A}
    action_lists::Vector{Vector{A}}
end
const ActionIterState = Vector{Int}
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(ones(Int, length(it.action_lists)))
    if length(iter_state) > 0
        iter_state[end] = 0
        return iterate(it, iter_state)
    else
        return nothing
    end
end
function Base.iterate(iter::ActionIter,iter_state::ActionIterState)
    iter_state = deepcopy(iter_state)
    i = length(iter_state)
    while i > 0
        if iter_state[i] < length(iter.action_lists[i])
            iter_state[i] += 1
            action = Action([iter.action_lists[j][idx] for (j,idx) in enumerate(iter_state)])
            return action, iter_state
        else
            iter_state[i] = 1
            i -= 1
        end
    end
    return nothing
end
function CRCBS.get_possible_actions(env::E,s::State) where {E<:LowLevelEnv}
    return ActionIter([collect(get_possible_actions(env_,s_)) for (env_,s_) in zip(get_envs(env), s.states)])
end
Base.length(iter::ActionIter) = product([length(a) for a in iter.action_lists])
CRCBS.get_next_state(s::State,a::Action) = State(
    [get_next_state(s_,a_) for (s_,a_) in zip(s.states,a.actions)]
    )
CRCBS.get_next_state(env::E,s::State,a::Action) where {E<:AbstractMetaEnv} = State(
    [get_next_state(e_,s_,a_) for (e_,s_,a_) in zip(get_envs(env),s.states,a.actions)]
    )
CRCBS.get_transition_cost(env::E,s::State,a::Action,sp::State) where {E<:AbstractMetaEnv} = [
    get_transition_cost(e_,s_,a_,sp_) for (e_,s_,a_,sp_) in zip(get_envs(env),s.states,a.actions,sp.states)
]
function CRCBS.violates_constraints(env::E, s::State, a::Action, sp::State) where {E<:AbstractMetaEnv}
    for (i, (e_,s_,a_,sp_)) in enumerate(zip(get_envs(env),s.states,a.actions,sp.states))
        if violates_constraints(e_,s_,a_,sp_)
            return true
        end
        # check for self conflict
        P1 = PathNode(s_,a_,sp_)
        for (j, (s2_,a2_,sp2_)) in enumerate(zip(s.states,a.actions,sp.states))
            if i != j
                P2 = PathNode(s2_,a2_,sp2_)
                if detect_state_conflict(P1,P2) || detect_action_conflict(P1,P2)
                    return true
                end
            end
        end
    end
    return false
end

### Reservation Table Interface
function CRCBS.reserve!(table::PIBTReservationTable,env::MetaAgentCBS.AbstractMetaEnv,s,a,sp,t=-1)
    valid = true
    for args in zip(get_envs(env),s.states,a.actions,sp.states)
        valid &= reserve!(table,args...)
    end
    return valid
end
function CRCBS.is_reserved(table::PIBTReservationTable,env::MetaAgentCBS.AbstractMetaEnv,s,a,sp,t=-1)
    for args in zip(get_envs(env),s.states,a.actions,sp.states)
        if is_reserved(table,args...)
            return true
        end
    end
    return false
end
function CRCBS.create_reservations(env::AbstractMetaEnv,s,a,sp,t=-1)
    vcat(map(args->create_reservations(args...,t), zip(get_envs(env),s.states,a.actions,sp.states))...)
end

end
