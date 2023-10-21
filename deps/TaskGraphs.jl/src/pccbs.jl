# export PCCBS
#
# module PCCBS
#
# using CRCBS
# using Parameters, LightGraphs, DataStructures
# using GraphUtils
# using ..PlanningPredicates

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# const State = CRCBS.GraphState
@with_kw struct State <: CRCBS.AbstractGraphState
    vtx::Int        = -1
    t::Int          = -1
    active::Bool    = true
end
State(s::GraphState) = State(s.vtx,s.t)
Base.convert(::Type{State},s::GraphState) = State(s)
State(vtx,t) = State(vtx=vtx,t=t)
_state_active(s) = true
_state_active(s::State) = s.active
const Action = CRCBS.GraphAction

function CRCBS.detect_state_conflict(n1::N,n2::N) where {S<:State,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if _state_active(n1.sp) && _state_active(n2.sp)
        if get_vtx(n1.sp) == get_vtx(n2.sp) && get_t(n1.sp) == get_t(n2.sp)
            return true
        end
    end
    return false
end
function CRCBS.detect_action_conflict(n1::N,n2::N) where {S<:State,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if _state_active(n1.sp) && _state_active(n2.sp)
        if (get_e(n1.a).src == get_e(n2.a).dst) && (get_e(n1.a).dst == get_e(n2.a).src) && (get_t(n1.sp) == get_t(n2.sp))
            return true
        end
    end
    return false
end

export PCCBSEnv

@with_kw_noshow struct PCCBSEnv{E,N,I,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{State,Action,C}
    search_env::E                   = nothing
    schedule_node::N                = nothing
    node_id::I                      = nothing
    agent_idx::Int                  = -1
    constraints::T                  = discrete_constraint_table(search_env,agent_idx)
    goal::State                     = State()
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_cost_model(env::PCCBSEnv)       = env.cost_model
CRCBS.get_heuristic_model(env::PCCBSEnv)  = env.heuristic
GraphUtils.get_node(env::PCCBSEnv)        = env.schedule_node
GraphUtils.get_graph(env::PCCBSEnv)       = get_graph(env.search_env,
    graph_key(get_node(env))
    )

function Base.show(io::IO, env::PCCBSEnv)
    print(io,"PCCBSEnv: \n",
        "\t","schedule_node: ",string(get_node(env)),"\n",
        "\t","agent_idx:     ",get_agent_id(env),"\n",
        "\t","goal:          ",string(get_goal(env)),"\n")
end

CRCBS.get_next_state(s::State,a::Action) = State(get_e(a).dst,get_t(s)+get_dt(a),_state_active(s))
CRCBS.get_next_state(env::PCCBSEnv,s,a)  = get_next_state(s,a)
CRCBS.wait(env::PCCBSEnv,s)              = Action(e=Edge(get_vtx(s),get_vtx(s)))

CRCBS.get_possible_actions(env::PCCBSEnv,s::State) = get_possible_actions(get_node(env),env,s)
function CRCBS.get_possible_actions(node,env::PCCBSEnv,s::State)
    if CRCBS.is_valid(env,s)
        return map(v2->Action(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return Action[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::PCCBSEnv,s::State)
    # @assert get_vtx(s) == get_id(get_initial_location_id(node)) "Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))"
    if !(get_vtx(s) == get_id(get_initial_location_id(node)))
        @log_info(-1,0,"Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))")
    end
    if CRCBS.is_valid(env,s)
        return [CRCBS.wait(env,s)]
    end
    return Action[]
end
function CRCBS.get_possible_actions(env::MetaAgentCBS.TeamMetaEnv,s::MetaAgentCBS.State{State})
    d_set = Set{Tuple}([(0,0),(-1,0),(0,1),(1,0),(0,-1)])
    for (e,s) in zip(env.envs, s.states)
        intersect!(d_set, CRCBS.get_graph(e).edge_cache[s.vtx])
    end
    meta_actions = Vector{MetaAgentCBS.Action{Action}}()
    for d in d_set
        meta_action = MetaAgentCBS.Action{Action}()
        valid_action = true
        for (e,s) in zip(env.envs, s.states)
            vtx = get_graph(e).vtxs[s.vtx]
            next_vtx = (vtx[1] + d[1], vtx[2] + d[2])
            edge = Edge(get_graph(e).vtx_map[vtx...],get_graph(e).vtx_map[next_vtx...])
            if has_edge(get_graph(e),edge)
                push!(meta_action.actions, Action(e = edge))
            else
                valid_action = false
                break
            end
        end
        if valid_action
            push!(meta_actions, meta_action)
        end
    end
    meta_actions
end

function CRCBS.is_goal(env::PCCBSEnv,s)
    if get_t(s) >= get_t(get_goal(env))
        if states_match(s, get_goal(env))
            #########################################
            # updated goal time
            v = get_vtx(get_schedule(env.search_env),env.node_id)
            path_spec = get_path_spec(get_schedule(env.search_env), v)
            # goal_time = get_tF(env.search_env,v) # time after which goal can be satisfied
            goal_time = get_t(get_goal(env))
            if path_spec.tight == true
                goal_time = get_tF(env.search_env,v) + minimum(get_local_slack(env.search_env,v))
                # goal_time += minimum(get_local_slack(env.search_env,v))
            end
            if (path_spec.free == true) && is_terminal_node(get_graph(get_schedule(env.search_env)),v)
                goal_time = maximum(get_cache(env.search_env).tF)
            end
            # check that all predecessor nodes have been completed.
            if get_t(s) < goal_time
                return false
            end
            #########################################
            return true
        elseif !CRCBS.is_valid(get_goal(env))
            return true
        end
    end
    return false
end

# end # end module PCCBS
