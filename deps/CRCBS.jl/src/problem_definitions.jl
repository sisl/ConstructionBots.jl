export
    AbstractMAPF,
    MAPF,
    action_type,
    state_type,
    cost_type,
    num_agents,
    num_goals,
    get_env,
    get_starts,
    get_goals,
    get_start,
    get_goal,
    get_start

abstract type AbstractMAPF end

"""
    MAPF

A MAPF is a Multi Agent Path Finding problem. It consists of an environment,
`env`, through which a group of agents may navigate, as well as a list of
start and goal states in that environment. Note that this is the _labeled_
case, where each agent has a specific assigned destination.

Elements:
- env::E - the base environment
- starts::Vector{S} - the vector of initial states
- starts::Vector{G} - the vector of goals
"""
struct MAPF{E,S,G} <: AbstractMAPF # Multi Agent Path Finding Problem
    env     ::E           # Environment Type
    starts  ::Vector{S}   # Vector of initial states
    goals   ::Vector{G}   # Vector of goal states
end
get_env(mapf::MAPF)             = mapf.env
action_type(mapf::MAPF)         = action_type(get_env(mapf))
state_type(mapf::MAPF)          = state_type(get_env(mapf))
cost_type(mapf::MAPF)           = cost_type(get_env(mapf))
num_agents(mapf::MAPF)          = length(mapf.starts)
num_goals(mapf::MAPF)           = length(mapf.goals)
get_starts(mapf::MAPF)          = mapf.starts
get_goals(mapf::MAPF)           = mapf.goals
get_start(mapf::MAPF, i)        = get_starts(mapf)[i]
get_goal(mapf::MAPF, i)         = get_goals(mapf)[i]
get_start(mapf::MAPF, env, i)   = get_start(mapf,i)
base_env_type(mapf::MAPF)       = base_env_type(mapf.env)
base_env_type(env)              = typeof(env)
# TODO implement a check to be sure that no two agents have the same goal
get_initial_cost(mapf::M) where {M<:AbstractMAPF}           = get_initial_cost(mapf.env)
get_infeasible_cost(mapf::M) where {M<:AbstractMAPF}        = get_infeasible_cost(mapf.env)
GraphUtils.get_distance(mapf::AbstractMAPF,args...) = get_distance(mapf.env,args...)
