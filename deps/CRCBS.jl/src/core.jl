export
    PathNode,
    get_s,
    get_a,
    get_sp

"""
    PathNode{S,A}

Includes current state `s`, action `a`, next state `sp`
"""
@with_kw struct PathNode{S,A}
    s::S = S() # state
    a::A = A() # action
    sp::S = S() # next state
end
"""
    get_s

Get the first state in a `PathNode`.
"""
get_s(p::P) where {P<:PathNode} = p.s
"""
    get_a

Get the action in a `PathNode`.
"""
get_a(p::P) where {P<:PathNode} = p.a
"""
    get_sp

Get the next state in a `PathNode`.
"""
get_sp(p::P) where {P<:PathNode} = p.sp
state_type(p::PathNode{S,A}) where {S,A} = S
action_type(p::PathNode{S,A}) where {S,A} = A
Base.string(p::PathNode) = "$(string(get_s(p))) -- $(string(get_a(p))) -- $(string(get_sp(p)))"

export
    Path,
    node_type,
    get_cost,
    set_cost!,
    get_initial_state,
    get_final_state,
    get_start_time,
    get_end_time,
    get_path_node,
    extend_path!,
    extend_path

abstract type AbstractPath end

"""
    Path{S,A,C}

Encodes a motion plan as a sequence of `PathNode{S,A}`s
"""
@with_kw mutable struct Path{S,A,C} <: AbstractPath
    path_nodes  ::Vector{PathNode{S,A}} = Vector{PathNode{S,A}}()
    s0          ::S                     = get_s(get(path_nodes, 1, PathNode{S,A}()))
    cost        ::C                     = typemax(C)
end
Path(v::Vector{P}) where {P<:PathNode}      = Path(path_nodes=v,s0=get_s(get(v,1,P())),cost=0.0)
state_type(p::Path{S,A,C}) where {S,A,C}    = S
action_type(p::Path{S,A,C}) where {S,A,C}   = A
node_type(p) = PathNode{state_type(p),action_type(p)}
cost_type(p::Path{S,A,C}) where {S,A,C}     = C
Base.cat(p::P,x::N,i...) where {P<:Path,N<:PathNode} = P(s0=p.s0,path_nodes=cat(p.path_nodes,x,dims=1),cost=p.cost)
Base.get(p::P,i,default=node_type(p)) where {P<:Path}    = get(p.path_nodes,i,default)
Base.getindex(p::P,i) where {P<:Path}       = getindex(p.path_nodes,i)
Base.setindex!(p::P,x,i) where {P<:Path}    = setindex!(p.path_nodes,x,i)
Base.length(p::P) where {P<:Path}           = length(p.path_nodes)
function Base.push!(p::P,n::N) where {P<:Path,N<:PathNode}
    if length(p.path_nodes) == 0
        p.s0 = n.s
    end
    push!(p.path_nodes,n)
end
get_cost(p::P) where {P<:Path}              = p.cost
function set_cost!(p::P,cost) where {P<:Path}
    p.cost = cost
end
Base.copy(p::P) where {P<:Path}             = Path(s0=p.s0,path_nodes=copy(p.path_nodes),cost=p.cost)

function get_initial_state(path::P) where {P<:Path}
    if length(path) > 0
        s0 = get_s(get(path,1,node_type(path)()))
    else
        s0 = path.s0
    end
    # @assert get_t(s0) == 0
    return s0
end
function get_final_state(path::P) where {P<:Path}
    if length(path) > 0
        sf = get_sp(get(path,length(path),node_type(path)()))
    else
        sf = path.s0
    end
    # @assert get_t(sf) == length(path) "length(path) = $(length(path)), but get_t(get_final_state(path)) = $(get_t(sf))"
    return sf
end


export
    TimeIndexedState,
    get_start_index,
    get_index_from_time,
    get_end_index

get_start_index(path::P) where {P<:AbstractPath} = 0
get_end_index(path::P) where {P<:AbstractPath} = length(path) + get_start_index(path)
get_index_from_time(path::P,t::Int) where {P<:AbstractPath} = t - get_start_index(path)
abstract type TimeIndexedState end
get_start_index(path::P) where {S<:TimeIndexedState,A,C,P<:Path{S,A,C}} = 1 - get_time_index(path.s0)

"""
returns the `PathNode` (s,a,s') corresponding to step `t` of `path`

If `t` is greater than the length of `path`, the `PathNode` returned
is (s,wait(s),s) corresponding to waiting at that node of the path.

path[t] is the path node that begins at t-1 and terminates at t
"""
function get_path_node(path::P,t::Int) where {P<:AbstractPath}
    t_idx = get_index_from_time(path,t)
    if 1 <= t_idx <= length(path)
        return path[t_idx]
    elseif t_idx == 0
        return node_type(path)(sp=get_initial_state(path))
    else
        t₀ = get_index_from_time(path,get_end_index(path))
        if t₀ <= 0
            node = node_type(path)(sp=get_initial_state(path))
        else
            node = get(path,length(path),node_type(path)(s=get_initial_state(path)))
        end
        s = get_s(node)
        a = get_a(node)
        sp = get_sp(node)
        for τ in t₀+1:t_idx
            s = sp
            a = wait(s)
            sp = get_next_state(s,a)
        end
        return PathNode(s,a,sp)
    end
end
function get_path_node(path::P,t::Float64) where {P<:AbstractPath}
    @assert abs(t - Int(round(t))) < 0.01
    get_path_node(path,Int(round(t)))
end

# """
#     `get_a(path,t)`
#
#     Returns the action at time `t`.
#
#     If `t` is greater than the length of `path`, the returned action defaults to
#     a `wait` action.
# """
get_s(path::P, t::Int) where {P<:Path} = get_s(get_path_node(path,t))
get_a(path::P, t::Int) where {P<:Path} = get_a(get_path_node(path,t))
get_sp(path::P, t::Int) where {P<:Path} = get_sp(get_path_node(path,t))


"""
    extend_path!(path,T)

Extends `path` to match a given length `T` by adding `PathNode`s
corresponding to waiting at the final state.

args:
- path      the path to be extended
- the desired length of the new path
"""
function extend_path!(path::P,T::Int) where {P<:Path}
    # while length(path) < T
    # while get_index_from_time(path,get_end_index(path)) < T
    while get_end_index(path) < T
        s = get_final_state(path)
        a = wait(s)
        push!(path,PathNode(s,wait(s),get_next_state(s,a)))
    end
    return path
end

"""
    extend_path(path,T)

Extends a copy of `path` to match a given length `T` by adding `PathNode`s
corresponding to waiting at the final state.

args:
- path      the path to be extended
- the desired length of the new path
"""
function extend_path(path::P,args...) where {P<:Path}
    new_path = copy(path)
    extend_path!(new_path,args...)
    return new_path
end

export
    AbstractCostModel,
    cost_type

"""
    AbstractCostModel{T}
"""
abstract type AbstractCostModel{T} end
cost_type(model::M) where {T,M<:AbstractCostModel{T}} = T

export
    LowLevelSolution,
    path_type,
    get_paths,
    get_path_costs,
    set_solution_path!,
    set_path_cost!

"""
    LowLevelSolution{S,A,T,C}

Contains a list of agent paths and the associated costs.
Params:
- `S` is the state type
- `A` is the action type
- `T` is the cost type
- `C` is the cost model type
Elements:
- `paths::Vector{Path{S,A,T}}` is the vector of paths
- `costs::Vector{T}` is the vector of costs, one per path
- `cost::T` is the total cost for the entire solution
"""
@with_kw_noshow mutable struct LowLevelSolution{S,A,T,C<:AbstractCostModel{T}}
    paths::Vector{Path{S,A,T}}  = Vector{Path{S,A,T}}()
    cost_model::C               = C() # TODO C() is a problem
    costs::Vector{T}            = Vector{cost_type(cost_model)}(map(i->get_initial_cost(cost_model),1:length(paths)))
    cost::T                     = get_initial_cost(cost_model)
end
state_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}    = S
action_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}   = A
cost_type(s::LowLevelSolution{S,A,T,C}) where {S,A,T,C}     = T
path_type(s) = Path{state_type(s),action_type(s),cost_type(s)}
Base.copy(solution::L) where {L <: LowLevelSolution} = L(
    paths=copy(solution.paths), # NOTE don't want to needlessly copy paths between search nodes
    cost_model=deepcopy(solution.cost_model),
    costs=copy(solution.costs),
    cost=deepcopy(solution.cost),
    )
get_paths(solution::L) where {L <: LowLevelSolution}        = solution.paths
get_path_costs(solution::L) where {L <: LowLevelSolution}   = solution.costs
get_cost(solution::L) where {L <: LowLevelSolution}         = solution.cost
get_cost_model(solution::L) where {L <: LowLevelSolution}   = solution.cost_model
function set_solution_path!(solution::L, path::P, idx::Int) where {L<:LowLevelSolution, P<:Path}
    solution.paths[idx] = path
    return solution
end
function set_path_cost!(solution::L, cost::C, idx::Int) where {L<:LowLevelSolution,C}
    solution.costs[idx] = cost
    solution.paths[idx].cost = cost
    return solution
end
function set_cost!(solution::L,cost) where {L<:LowLevelSolution}
    solution.cost = cost
    return solution
end

export
    AbstractLowLevelEnv,
    action_type,
    state_type,
    get_cost_model,
    get_heuristic_model,
    cost_type

"""
    AbstractLowLevelEnv{S,A,C}

Defines a prototype environment for low level search (searching for a path
for a single agent).

`S` is the State type, `A` is the action type, and `C` is the cost type. All
three must be default constructible (i.e. you can call `S()`, `A()` and `C()`
without throwing errors)

In general, a concrete subtype of `AbstractLowLevelEnv` may include a graph
whose edges are traversed by agents.
"""
abstract type AbstractLowLevelEnv{S,A,C} end
action_type(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = A
state_type(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = S
cost_type(env::E) where {E<:AbstractLowLevelEnv} = cost_type(get_cost_model(env))
""" Override this method for when the cost model has arguments """
get_cost_model(env::E) where {S,A,C,E<:AbstractLowLevelEnv{S,A,C}} = C()
function get_heuristic_model end


export
    build_env,
    states_match,
    is_goal,
    wait,
    get_possible_actions,
    get_next_state,
    get_transition_cost,
    get_path_cost,
    get_heuristic_cost,
    violates_constraints,
    check_termination_criteria

################################################################################
######################## General Methods to Implement ##########################
################################################################################

"""
    build_env(mapf::AbstractMAPF, node::ConstraintTreeNode, idx::Int)

Constructs a new low-level search environment for a conflict-based search
mapf solver
"""
function build_env end

"""
    state_match(s1::S,s2::S)

returns true if s1 and s2 match (not necessarily the same as equal)
"""
function states_match end

"""
    is_goal(env,s)

Returns true if state `s` satisfies the goal condition of environment `env`
"""
function is_goal end

"""
    wait(s)

returns an action that corresponds to waiting at state s
"""
function wait end

"""
    get_possible_actions(env::E <: AbstractLowLevelEnv{S,A,C}, s::S)

return type must support iteration
"""
function get_possible_actions end

"""
    get_next_state(env::E <: AbstractLowLevelEnv{S,A,C}, s::S, a::A)

returns a next state s
"""
function get_next_state end

"""
    get_transition_cost(env::E <: AbstractLowLevelEnv{S,A,C},s::S,a::A,sp::S)
    get_transition_cost(env,s,a)
    get_transition_cost(h,env,s,a,p)

return scalar cost for transitioning from `s` to `sp` via `a`
"""
function get_transition_cost end

"""
    get_path_cost(env::E <: AbstractLowLevelEnv{S,A,C},path::Path{S,A,C})

get the cost associated with a search path so far
"""
function get_path_cost end

"""
    get_heuristic_cost(env::E <: AbstractLowLevelEnv{S,A,C},state::S)
    get_heuristic_cost(h,env,s)
    get_heuristic_cost(h,s)

get a heuristic "cost-to-go" from `state`
"""
function get_heuristic_cost end

"""
    violates_constraints(env::E <: AbstractLowLevelEnv{S,A,C},
        path::Path{S,A,C},s::S,a::A,sp::S)

returns `true` if taking action `a` from state `s` violates any constraints
    associated with `env`
"""
function violates_constraints end

"""
    check_termination_criteria(env::E <: AbstractLowLevelEnv{S,A,C}, cost,
        path::Path{S,A,C}, s::S)

returns true if any termination criterion is satisfied
"""
function check_termination_criteria end