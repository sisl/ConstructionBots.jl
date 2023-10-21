export
    MultiStageCBS

module MultiStageCBS

using ..CRCBS
using Parameters, Graphs, DataStructures
using GraphUtils: get_graph, get_vtx

# NOTE MultiStageCBS Could also serve as the definition for "iterative MAPF" as
# defined in "Priority Inheritance with Backtracking for Iterative Multi-agent
# Path Finding", Okumura et al, IJCAI 2019

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# state
@with_kw struct State <: AbstractGraphState
    vtx::Int    = -1 # vertex of graph
    stage::Int  = -1 # which stage of the sequence
    t::Int      = -1
end
Base.convert(::Type{State},s::GraphState) = State(vtx=get_vtx(s),t=get_t(s))
# CRCBS.get_vtx(s::State) = s.vtx
CRCBS.get_t(s::State) = s.t
get_stage(s::State) = s.stage
Base.string(s::State) = "(v=$(get_vtx(s)),stage=$(get_stage(s)),t=$(get_t(s)))"
CRCBS.is_valid(state::State) = get_vtx(state) > 0
# action
@with_kw struct Action <: AbstractGraphAction
    e::Edge{Int}    = Edge(-1,-1)
    dt::Int         = 1
end
Base.convert(::Type{Action},a::GraphAction) = Action(e=get_e(a),dt=get_dt(a))
CRCBS.get_e(a::Action) = a.e
CRCBS.get_dt(a::Action) = a.dt
Base.string(a::Action) = "(e=$(get_e(a).src) → $(get_e(a).dst))"
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:LowLevelSearchHeuristic,G<:AbstractGraph,T} <: GraphEnv{State,Action,C}
    graph::G                        = Graph()
    goal_sequence::Vector{State}    = Vector{State}()
    agent_idx::Int                  = -1
    constraints::T                  = discrete_constraint_table(nv(graph),nv(graph)^2,agent_idx)
    cost_model::C                   = SumOfTravelTime()
    heuristic::H                    = NullHeuristic() # MultiStagePerfectHeuristic(graph,Vector{Vector{Int}}())
end
# CRCBS.get_graph(env::LowLevelEnv)            = env.graph
# GraphUtils.get_graph(env::LowLevelEnv)            = env.graph
# CRCBS.get_cost_model(env::LowLevelEnv)       = env.cost_model
# CRCBS.get_agent_id(env::LowLevelEnv)         = env.agent_idx
# CRCBS.get_constraints(env::LowLevelEnv)      = env.constraints
# CRCBS.get_goal(env::LowLevelEnv)             = env.goal_sequence
# CRCBS.get_heuristic_model(env::LowLevelEnv)  = env.heuristic
CRCBS.base_env_type(env::LowLevelEnv)        = LowLevelEnv

function CRCBS.get_next_state(s::State,a::Action)
    @assert(is_valid(s))
    @assert(get_vtx(s) == get_e(a).src)
    State(get_e(a).dst, get_stage(s), get_t(s)+get_dt(a))
end
function CRCBS.get_next_state(env::E,s::State,a::Action) where {E<:LowLevelEnv}
    @assert(is_valid(s))
    @assert(get_stage(s) <= length(env.goal_sequence))
    stage = get_stage(s)
    if states_match(s, env.goal_sequence[get_stage(s)])
        stage = min(stage+1, length(env.goal_sequence))
    end
    return State(get_e(a).dst, stage, get_t(s)+get_dt(a))
end
CRCBS.get_possible_actions(env::LowLevelEnv,s)  = map(v->Action(e=Edge(get_vtx(s),v)),outneighbors(get_graph(env),get_vtx(s)))
CRCBS.wait(s::State) = Action(e=Edge(get_vtx(s),get_vtx(s)))
CRCBS.wait(env::E,s::State) where {E<:LowLevelEnv} = Action(e=Edge(get_vtx(s),get_vtx(s)))

function CRCBS.build_env(mapf::MAPF{E,S,G}, node::N, idx::Int)  where {S,G,E <: LowLevelEnv,N<:ConstraintTreeNode}
    goals = deepcopy(mapf.goals[idx])
    n = PathNode{State,Action}(sp=goals[end])
    s_constraints, _ = search_constraints(mapf.env,get_constraints(node,idx),n)
    t_goal = get_t(get_sp(n))
    for c in s_constraints
        t_goal = max(t_goal,get_time_of(c)+1)
    end
    goals[end] = State(get_sp(n),t=t_goal)
    E(
        graph = get_graph(mapf.env),
        constraints = get_constraints(node,idx),
        goal_sequence = goals,
        agent_idx = idx,
        cost_model = get_cost_model(mapf.env),
        heuristic = get_heuristic_model(mapf.env)
        )
end
function CRCBS.is_consistent(path::Path,start::State,goals::Vector{State})
    valid = true
    stage = 1
    if states_match(start,goals[stage])
        stage += 1
    end
    for k in 1:length(path)
        node = get_path_node(path,k)
        if states_match(get_sp(node),goals[stage])
            stage += 1
        end
        if stage > length(goals)
            return true
        end
    end
    return false
end
function CRCBS.is_goal(env::E,s::State) where {E<:LowLevelEnv}
    if get_stage(s) == length(env.goal_sequence)
        if states_match(s, env.goal_sequence[get_stage(s)])
            if get_t(s) >= get_t(env.goal_sequence[get_stage(s)])
                return true
            end
        end
    end
    return false
end
function CRCBS.get_heuristic_cost(h::MultiStagePerfectHeuristic,env::E,s::State) where {E<:LowLevelEnv}
# function CRCBS.get_heuristic_cost(env::E,h::MultiStagePerfectHeuristic,s::State) where {E<:LowLevelEnv}
    get_heuristic_cost(h, env.agent_idx, s.stage, s.vtx)
end

end # end module MultiStageCBS
