import GraphUtils: get_graph, get_vtx

export
    AbstractGraphState,
    GraphState,
    # get_vtx,
    get_t,
    AbstractGraphAction,
    GraphAction,
    get_e,
    get_dt

abstract type AbstractGraphState end
@with_kw_noshow struct GraphState <: AbstractGraphState
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
get_vtx(s::AbstractGraphState)  = s.vtx
get_t(s::AbstractGraphState)    = s.t
is_valid(s::AbstractGraphState) = get_vtx(s) >= 1
Base.string(s::AbstractGraphState) = "(v=$(get_vtx(s)),t=$(get_t(s)))"
Base.show(io::IO,s::AbstractGraphState) = print(io,typeof(s),string(s)) #"(v=$(get_vtx(s)),t=$(get_t(s)))"
# allow paths to start at non-zero start times
get_start_index(path::Path{S,A,C}) where {S<:AbstractGraphState,A,C} = get_t(get_initial_state(path))

# AbstractGraphAction
export AbstractGraphAction

abstract type AbstractGraphAction end
@with_kw_noshow struct GraphAction <: AbstractGraphAction
    e::Edge{Int}    = Edge(-1,-1)
    dt::Int         = 1
end
get_e(a::AbstractGraphAction)   = a.e
get_dt(a::AbstractGraphAction)  = a.dt
Base.reverse(a::AbstractGraphAction) = typeof(a)(a,e=reverse(a.e))
Base.string(a::AbstractGraphAction) = "(e=$(get_e(a).src) → $(get_e(a).dst))"
Base.show(io::IO,a::AbstractGraphAction) = print(io,typeof(a),string(a)) #"(v=$(get_vtx(s)),t=$(get_t(s)))"

export
    GraphEnv

"""
    GraphEnv{S,A,C} <: AbstractLowLevelEnv{S,A,C}

An abstract environment type whose concrete subtypes comply with a standard 
interface.
"""
abstract type GraphEnv{S,A,C} <: AbstractLowLevelEnv{S,A,C} end

export
    # get_graph,
    get_cost_model,
    get_agent_id,
    get_constraints,
    get_goal,
    get_heuristic_model

"""
    GraphUtils.get_graph(env::GraphEnv)

Must be implemented for all concrete subtypes of `GraphEnv`
"""
get_graph(env::GraphEnv)            = env.graph
get_cost_model(env::GraphEnv)       = env.cost_model
get_agent_id(env::GraphEnv)         = env.agent_idx
get_constraints(env::GraphEnv)      = env.constraints
get_goal(env::GraphEnv)             = env.goal
get_heuristic_model(env::GraphEnv)  = env.heuristic
function GraphUtils.get_distance(env::GraphEnv,s1::AbstractGraphState,s2::AbstractGraphState)
    get_distance(get_graph(env),get_vtx(s1),get_vtx(s2))
end

set_agent_idx!(env::E,i) where {E<:GraphEnv} = E(env,agent_idx=i)

cost_type(env::GraphEnv)            = cost_type(get_cost_model(env))

is_valid(env::GraphEnv,s::AbstractGraphState) = has_vertex(get_graph(env),get_vtx(s))
is_valid(env::GraphEnv,a::AbstractGraphAction) = has_edge(get_graph(env),get_e(a))
# get_possible_actions(env::GraphEnv,s) = map(v->GraphAction(e=Edge(get_vtx(s),v)),outneighbors(get_graph(env),get_vtx(s)))
get_next_state(s::AbstractGraphState,a::AbstractGraphAction) = GraphState(get_e(a).dst,get_t(s)+get_dt(a))
# get_next_state(env::GraphEnv,s,a) = get_next_state(s,a)
wait(s::AbstractGraphState) = GraphAction(e=Edge(get_vtx(s),get_vtx(s)))
# wait(env::GraphEnv,s) = GraphAction(e=Edge(get_vtx(s),get_vtx(s)))
function get_transition_cost(c::TravelTime,env::GraphEnv,s,a,sp)
    return cost_type(c)(get_dt(a))
end
function get_conflict_value(c::C,agent_id::Int,env::GraphEnv,s::AbstractGraphState,t=get_t(s)) where {C<:SoftConflictTable}
    idx,t = serialize_jointly(env,s,t)
    get_conflict_value(c,agent_id,idx,t)
end
function get_conflict_value(c::C,agent_id::Int,env::GraphEnv,a::AbstractGraphAction,t) where {C<:SoftConflictTable}
    idx,t = serialize_jointly(env,a,t)
    get_conflict_value(c,agent_id,idx,t)
end
function get_transition_cost(c::C,env::GraphEnv,s,a,sp) where {C<:ConflictCostModel}
    state_conflict_value = get_conflict_value(c, get_agent_id(env), get_vtx(sp), get_t(sp))
    edge_conflict_value = min(
        get_conflict_value(c,get_agent_id(env), get_vtx(sp), get_t(s)),
        get_conflict_value(c,get_agent_id(env), get_vtx(s), get_t(sp)),
        )
    if edge_conflict_value > 0
        # println("Possible Edge Conflict")
        edge_conflict_value = 0
        for (i,p) in enumerate(c.table.paths)
            if (get_planned_vtx(c.table, i, get_t(s)) == get_vtx(sp)) && (get_planned_vtx(c.table, i, get_t(sp)) == get_vtx(s))
                edge_conflict_value += 1
            end
        end
    end
    return state_conflict_value + edge_conflict_value
end
function get_transition_cost(c::ConflictCostModel{S},env::GraphEnv,s,a,sp) where {S<:SoftConflictTable}
    state_conflict_value = get_conflict_value(c.table,get_agent_id(env),env,sp)
    action_conflict_value = get_conflict_value(c.table,get_agent_id(env),env,a,get_t(sp))
    return state_conflict_value + action_conflict_value
end
function get_transition_cost(c::TravelDistance,env::GraphEnv,s,a,sp)
    return (get_vtx(s) == get_vtx(sp)) ? 0.0 : 1.0
end
compute_path_cost(c::TravelTime,env::GraphEnv,path,i) = cost_type(c)(get_t(get_final_state(path)))
function compute_path_cost(c::TravelDistance,env::GraphEnv,path,i)
    cost_type(c)(sum(map(
        n->get_e(get_a(n)).src != get_e(get_a(n)).dst,
        path.path_nodes)))
end
# get_heuristic_cost(env::GraphEnv,s) = get_heuristic_cost(env,get_heuristic_model(env),s)
get_heuristic_cost(env::GraphEnv,s) = get_heuristic_cost(get_heuristic_model(env),env,s)
# function get_heuristic_cost(env::GraphEnv,h::H,s) where {H<:Union{PerfectHeuristic,DefaultPerfectHeuristic}}
function get_heuristic_cost(h::H,env::GraphEnv,s) where {H<:Union{PerfectHeuristic,DefaultPerfectHeuristic}}
    get_heuristic_cost(h, get_vtx(get_goal(env)), get_vtx(s))
end
# function get_heuristic_cost(env::GraphEnv,h::EnvDistanceHeuristic,s)
function get_heuristic_cost(h::EnvDistanceHeuristic,env::GraphEnv,s)
    get_distance(env, s, get_goal(env))
end
# function get_heuristic_cost(env::GraphEnv,h::H,s) where {E<:GraphEnv, H<:ConflictTableHeuristic}
function get_heuristic_cost(h::H,env::GraphEnv,s) where {E<:GraphEnv, H<:ConflictTableHeuristic}
    get_heuristic_cost(h, get_agent_id(env), get_vtx(s), get_t(s))
end
# states_match
states_match(s1::AbstractGraphState,s2::AbstractGraphState) = (get_vtx(s1) == get_vtx(s2))
states_match(env::GraphEnv,s1,s2) = (get_vtx(s1) == get_vtx(s2))
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
num_states(env::GraphEnv)                           = nv(get_graph(env))
num_actions(env::GraphEnv)                          = num_states(env)^2 # NOT actually true, but necessary for O(1) serialization
state_space_trait(env::GraphEnv)                    = DiscreteSpace()
action_space_trait(env::GraphEnv)                   = DiscreteSpace()
serialize(env::GraphEnv,s::AbstractGraphState,t=get_t(s))            = get_vtx(s), t
# NOTE the return type of deserialize doesn't need to match the state/action type of env, since it is only used for constraint checking
deserialize(env::GraphEnv,s::AbstractGraphState,idx::Int,t=-1) = GraphState(vtx=idx,t=t), t
function serialize(env::GraphEnv,a::AbstractGraphAction,t=-1)
    (get_e(a).src-1)*num_states(env)+get_e(a).dst, t
end
function deserialize(env::GraphEnv,s::AbstractGraphAction,idx::Int,t=get_t(s))
    GraphAction(e = Edge(
        div(idx-1,num_states(env))+1,
        mod(idx-1,num_states(env))+1)
        ), t
end
serialize_jointly(env::GraphEnv,s::AbstractGraphState,t=get_t(s)) = serialize(env,s,t)
function serialize_jointly(env::GraphEnv,a::A,t=-1) where {A<:AbstractGraphAction}
    a_ = A(a,e=Edge(sort([get_e(a).src, get_e(a).dst])...))
    idx, t = serialize(env,a_,t)
    return idx+num_states(env), t
end

"""
    create_reservations(env::GraphEnv,s,a,sp,t=-1)

Generates three reservations as follows
```
t0 = get_t(s)
tF = get_t(sp)
t_mid = (t0+tF)/2
reservations = [
    ResourceReservation{Int}(s_idx,get_agent_id(env),   (t0,    t_mid)),
    ResourceReservation{Int}(a_idx,get_agent_id(env),   (t0,    tF)),
    ResourceReservation{Int}(sp_idx,get_agent_id(env),  (t_mid, tF)),
]
```
In this way, the reservations for one path node will not interfere with those
for the next path node.
"""
function create_reservations(env::GraphEnv,s,a,sp,t=-1)
    s_idx,t0    = serialize(env,s)
    a_idx,_     = serialize_jointly(env,a) # serialize action to not overlap with state serialization space?
    sp_idx,tF   = serialize(env,sp)
    t_mid = (t0+tF)/2
    reservations = [
        ResourceReservation{Float64}(s_idx,get_agent_id(env),   (t0,    t_mid)),
        ResourceReservation{Float64}(a_idx,get_agent_id(env),   (t0,    tF)),
        ResourceReservation{Float64}(sp_idx,get_agent_id(env),  (t_mid, tF)),
    ]
end
function create_reservations(env::GraphEnv,n::PathNode,t=-1)
    create_reservations(env,get_s(n),get_a(n),get_sp(n),t)
end
# is_goal
function is_goal(env::GraphEnv,s)
    if get_t(s) >= get_t(get_goal(env))
        if states_match(s, get_goal(env)) || !is_valid(get_goal(env))
            return true
        end
    end
    return false
end
function violates_constraints(env::GraphEnv, s, a, sp)
    t = get_t(sp)
    if has_constraint(env,get_constraints(env),
        state_constraint(get_agent_id(get_constraints(env)),PathNode(s,a,sp),t)
        )
        return true
    elseif has_constraint(env,get_constraints(env),
        action_constraint(get_agent_id(get_constraints(env)),PathNode(s,a,sp),t)
        )
        return true
    end
    return false
end

################################################################################
###################### Conflict-Based Search (High-Level) ######################
################################################################################
function detect_state_conflict(n1::N,n2::N) where {S<:AbstractGraphState,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if get_vtx(n1.sp) == get_vtx(n2.sp) && get_t(n1.sp) == get_t(n2.sp)
        return true
    end
    return false
end
function detect_action_conflict(n1::N,n2::N) where {S<:AbstractGraphState,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if (get_e(n1.a).src == get_e(n2.a).dst) && (get_e(n1.a).dst == get_e(n2.a).src) && (get_t(n1.sp) == get_t(n2.sp))
        return true
    end
    return false
end

################################################################################
############################### HELPER FUNCTIONS ###############################
################################################################################
""" Helper for displaying Paths """
function convert_to_vertex_lists(path::Path{S,A,C}) where {S<:AbstractGraphState,A<:AbstractGraphAction,C}
    vtx_list = [get_vtx(n.sp) for n in path.path_nodes]
    # if length(path) > 0
    #     vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    # else
        vtx_list = [get_vtx(get_initial_state(path)), vtx_list...]
    # end
    vtx_list
end
