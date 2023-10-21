export
    CBSEnv

# CBS submodule
module CBSEnv

using ..CRCBS
using Parameters, Graphs, DataStructures
using GraphUtils: get_graph, get_vtx

@with_kw struct State <: AbstractGraphState
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
Base.convert(::Type{State},s::GraphState) = State(vtx=get_vtx(s),t=get_t(s))
# CRCBS.get_vtx(s::State) = s.vtx
CRCBS.get_t(s::State) = s.t
@with_kw struct Action <: AbstractGraphAction
    e::Edge{Int}    = Edge(-1,-1)
    dt::Int         = 1
end
Base.convert(::Type{Action},a::GraphAction) = Action(e=get_e(a),dt=get_dt(a))
CRCBS.get_e(a::Action) = a.e
CRCBS.get_dt(a::Action) = a.dt
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:AbstractCostModel,G<:AbstractGraph,T} <: GraphEnv{State,Action,C}
    graph::G                    = Graph()
    goal::State                 = State()
    agent_idx::Int              = -1
    constraints::T              = discrete_constraint_table(nv(graph),nv(graph)^2,agent_idx)
    cost_model::C               = SumOfTravelTime()
    heuristic::H                = NullHeuristic() #PerfectHeuristic(graph,Vector{Int}(),Vector{Int}())
end
# CRCBS.get_graph(env::LowLevelEnv)            = env.graph
# GraphUtils.get_graph(env::LowLevelEnv)            = env.graph
# CRCBS.get_agent_id(env::LowLevelEnv)         = env.agent_idx
# CRCBS.get_constraints(env::LowLevelEnv)      = env.constraints
# CRCBS.get_goal(env::LowLevelEnv)             = env.goal
# CRCBS.get_cost_model(env::LowLevelEnv)       = env.cost_model
# CRCBS.get_heuristic_model(env::LowLevelEnv)  = env.heuristic
CRCBS.base_env_type(env::LowLevelEnv)        = LowLevelEnv

CRCBS.get_possible_actions(env::LowLevelEnv,s)  = map(v->Action(e=Edge(get_vtx(s),v)),outneighbors(get_graph(env),get_vtx(s)))
CRCBS.get_next_state(s::State,a::Action)    = State(get_e(a).dst,get_t(s)+get_dt(a))
CRCBS.get_next_state(env::LowLevelEnv,s,a)  = get_next_state(s,a)
CRCBS.wait(s::State)                        = Action(e=Edge(get_vtx(s),get_vtx(s)))
CRCBS.wait(env::LowLevelEnv,s)              = Action(e=Edge(get_vtx(s),get_vtx(s)))

function CRCBS.build_env(mapf::MAPF{E,S,G}, node::ConstraintTreeNode, idx::Int) where {S,G,E<:LowLevelEnv}
    if idx > 0
        t_goal = -1
        n = PathNode{State,Action}(sp=mapf.goals[idx])
        s_constraints, _ = search_constraints(mapf.env,get_constraints(node,idx),n)
        for c in s_constraints
            t_goal = max(t_goal,get_time_of(c)+1)
        end
        return typeof(mapf.env)(
            graph = get_graph(mapf.env),
            constraints = get_constraints(node,idx),
            goal = State(mapf.goals[idx],t=t_goal),
            agent_idx = idx,
            cost_model = get_cost_model(mapf.env),
            heuristic = get_heuristic_model(mapf.env),
            )
    else
        return typeof(mapf.env)(
            graph = get_graph(mapf.env),
            goal = state_type(mapf)(),
            agent_idx = idx,
            cost_model = get_cost_model(mapf.env),
            heuristic = get_heuristic_model(mapf.env),
            )
    end
end

end
