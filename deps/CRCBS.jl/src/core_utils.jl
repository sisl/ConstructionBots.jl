################################################################################
######################### LowLevelSolution Constructors ########################
################################################################################
export
    get_initial_solution,
    get_infeasible_solution,
    default_solution

"""
    get_initial_solution
"""
function get_initial_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    LowLevelSolution{S,A,T,C}(
        paths = map(i->Path{S,A,T}(
                    s0=get_start(mapf,i),
                    cost=get_initial_cost(mapf.env)),
                1:num_agents(mapf)),
        cost_model = get_cost_model(mapf.env),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_initial_cost(mapf.env),
    )
end
"""
    get_infeasible_solution
"""
function get_infeasible_solution(mapf::MAPF{E,S,G}) where {S,A,G,T,C<:AbstractCostModel{T},E<:AbstractLowLevelEnv{S,A,C}}
    LowLevelSolution{S,A,T,C}(
        paths = Vector{Path{S,A,T}}(map(a->Path{S,A,T}(cost=get_initial_cost(mapf.env)),1:num_agents(mapf))),
        cost_model = get_cost_model(mapf.env),
        costs = Vector{T}(map(a->get_initial_cost(mapf.env),1:num_agents(mapf))),
        cost = get_infeasible_cost(mapf.env),
    )
end
"""
    default_solution(solver, mapf::AbstractMAPF)

Defines what is returned by the solver in case of failure to find a feasible
solution.
"""
function default_solution(mapf::M) where {M<:AbstractMAPF}
    return get_infeasible_solution(mapf), get_infeasible_cost(mapf.env)
end

export add_to_path!

"""
    add_to_path!(path,env,s,a,sp)

Adds the new (s,a,sp) tuple and its cost (under env) to path.
"""
function add_to_path!(path::Path,env,s,a,sp)
    push!(path,PathNode(s,a,sp))
    set_cost!(path,accumulate_cost(env,get_cost(path),
        get_transition_cost(env,s,a,sp)))
    path
end

################################################################################
################################# Extend Paths #################################
################################################################################
export
    extend_path!

function extend_path!(env::E,path::P,T::Int) where {E<:AbstractLowLevelEnv,P<:Path}
    # while get_index_from_time(path,get_end_index(path)) < T
    while get_end_index(path) < T
        s = get_final_state(path)
        a = CRCBS.wait(env,s)
        sp = get_next_state(env,s,a)
        push!(path,PathNode(s,a,sp))
        set_cost!(path, accumulate_cost(env, get_cost(path), get_transition_cost(env,s,a,sp)))
    end
    return path
end
function extend_path!(env::E,path::P,T::Float64) where {E<:AbstractLowLevelEnv,P<:Path}
    @assert abs(T - Int(round(T))) < 0.01
    extend_path!(env,path,Int(round(T)))
end
function extend_path(env::E,path::P,args...) where {E<:AbstractLowLevelEnv,P<:Path}
    new_path = copy(path)
    extend_path!(new_path,args...)
    return new_path
end

get_transition_cost(env,n::PathNode) = get_transition_cost(env,get_s(n),get_a(n),get_sp(n))

export recompute_cost

"""
    recompute_cost

Recompute the cost of `path` (according to `env`), beginning from initial cost
`c0`.
"""
function recompute_cost(env,path,c0=get_initial_cost(env))
    cost = c0
    for n in path.path_nodes
        cost = accumulate_cost(env,cost,get_transition_cost(env,n))
    end
    return cost
end

export replace_cost_model

export trim_path!

"""
    trim_path!(env,path,T)

Modify `path` to terminate at time step `T`. If `length(path) < T`, the path
will be extended to time step `T`.
"""
function trim_path!(env,path,T)
    while length(path) > T
        # deleteat!(path.path_nodes,T)
        pop!(path.path_nodes)
    end
    if length(path) < T
        extend_path!(env,path,T)
    end
    set_cost!(path,recompute_cost(env,path))
    @assert length(path) == T
    return path
end

export trim_solution!

"""
    trim_solution!

Modify `solution` so that all paths terminate at time step `T`.
"""
function trim_solution!(env,solution,T)
    for (agent_id,path) in enumerate(get_paths(solution))
        trim_path!(env,path,T)
        set_path_cost!(solution,get_cost(path),agent_id)
    end
    set_cost!(solution,
        aggregate_costs(get_cost_model(env),get_path_costs(solution)))
    return solution
end

export sorted_actions

"""
    sorted_actions(env,s)

Return a vector of actions sorted lowest cost to highest cost.
"""
function sorted_actions(env,s)
    f = (s,a,sp)->compute_heuristic_cost(env,get_transition_cost(env,s,a,sp),sp)
    sort(
        collect(get_possible_actions(env,s)),
        by=a->f(s,a,get_next_state(env,s,a))
    )
end

################################################################################
################################## Utilities ###################################
################################################################################
export
    is_consistent,
    assert_valid,
    is_valid


"""
    is_consistent(solution,mapf)

    Check if solution satisfies start and end constraints
"""
function is_consistent(path::P,start::S,goal::G) where {S,G,P<:Path}
    return (states_match(get_initial_state(path), start)
        && states_match(get_final_state(path), goal))

end
function is_consistent(paths::Vector{P},starts::Vector{S},goals::Vector{G}) where {S,G,P <: Path}
    for (i,path) in enumerate(paths)
        if !is_consistent(path,starts[i],goals[i])
            return false
        end
    end
    return true
end
function is_consistent(solution::L,starts::Vector{S},goals::Vector{G}) where {S,G,L<:LowLevelSolution}
    is_consistent(get_paths(solution),starts,goals)
end
function is_consistent(solution::L,mapf::M) where {L<:LowLevelSolution, M<:AbstractMAPF}
    is_consistent(solution,get_starts(mapf),get_goals(mapf))
end
assert_valid(args...) = @assert(is_consistent(args...))
function is_valid(r)
    try
        assert_valid(r)
    catch e
        if isa(e,AssertionError)
           showerror(stdout,e)
           return false
        else
           rethrow(e)
        end
    end
    return true
end

################################################################################
######################### Visualization and Debugging ##########################
################################################################################

export
    convert_to_vertex_lists

function convert_to_vertex_lists(path) end
function convert_to_vertex_lists(solution::S) where {S<:LowLevelSolution}
    return [convert_to_vertex_lists(path) for path in get_paths(solution)]
end

export
    sprint_route_plan

function sprint_route_plan(io::IO,solution::LowLevelSolution;kwargs...)
    sprint_indexed_list_array(io::IO,convert_to_vertex_lists(solution);kwargs...)
end
function sprint_route_plan(route_plan::LowLevelSolution;kwargs...)
    buffer = IOBuffer()
    sprint_route_plan(buffer,route_plan;kwargs...)
    String(take!(buffer))
end
function Base.show(io::IO,route_plan::LowLevelSolution)
    # buffer = IOBuffer()
    # print(buffer,"LowLevelSolution:\n")
    print(io,"LowLevelSolution:\n")
    # sprint_route_plan(buffer,route_plan;leftaligned=true)
    sprint_route_plan(io,route_plan;leftaligned=true)
    # print(io,String(take!(buffer)))
end

export
    summarize

summarize(env) = ""
