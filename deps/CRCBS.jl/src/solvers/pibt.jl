export PIBTPlanner

"""
    PIBTPlanner{C}

Planner based on Priority Inheritance with BackTracking. 

Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding
Okumura et al, IJCAI 2019
https://www.ijcai.org/Proceedings/2019/0076.pdf
"""
@with_kw struct PIBTPlanner{C}
    logger::SolverLogger{C} = SolverLogger{C}()
    partial::Bool           = false # if true, allow partial solutions
end

export PIBTReservationTable

"""
    PIBTReservationTable
"""
struct PIBTReservationTable
    state_reservations::SparseVector{Bool,Int}
    action_reservations::SparseVector{Bool,Int}
end
function PIBTReservationTable(n_states::Int,n_actions::Int)
    PIBTReservationTable(
        sparse(zeros(Bool,n_states)),
        sparse(zeros(Bool,n_actions)),
        )
end
function PIBTReservationTable(env)
    PIBTReservationTable(num_states(env),num_actions(env))
end
function reserve_state!(table::PIBTReservationTable,env,s)
    idx,_ = serialize(env,s)
    table.state_reservations[idx] = true
    table
end
function reserve_action!(table::PIBTReservationTable,env,a)
    idx,_ = serialize(env,a)
    table.action_reservations[idx] = true
    table
end
function is_reserved(table::PIBTReservationTable,env,s,a,sp)
    s_idx,_ = serialize(env,sp)
    a_idx,_ = serialize(env,a)
    return table.state_reservations[s_idx] || table.action_reservations[a_idx]
end
function reserve!(table::PIBTReservationTable,env,s,a,sp)
    if is_reserved(table,env,s,a,sp)
        return false
    end
    reserve_state!(table,env,sp)
    reserve_action!(table,env,a)
    # table
    return true
end
function reset_reservations!(table::PIBTReservationTable)
    table.state_reservations .= false
    table.action_reservations .= false
    dropzeros!(table.state_reservations)
    dropzeros!(table.action_reservations)
    table
end





abstract type AbstractPIBTCache end

export PIBTCache

"""
    PIBTCache{S,A}

Contains info to be passed along through recursive calls to the PIBT algorithm
for multi-agent path planning.
Info to be stored:
- current state of each agent (should be lined up at the same time step)
- priority of each agent
- the planned action (and associated next state) for each agent
- the search environment for each agent, which contains e.g., the agent's goal,
    cost_model, heuristic_model, etc.
- a conflict table of sorts to indicate which states/actions are reserved
- countdown flags that identify which paths are "active". If pibt is operating
on a "ragged" plan, where some paths have been planned further into the future
than others, it needs to ensure that planning does not continue for a given path
until all of the other paths have "caught up" to it.
"""
struct PIBTCache{T,E,S,A} <: AbstractPIBTCache
    solution::T
    envs::Vector{E}
    states::Vector{S}
    actions::Vector{A}
    ordering::Vector{Int}
    undecided::Set{Int} # agent ids
    # reservation_table::Set{Int}
    # reservation_table::PIBTReservationTable
    reservation_table::ReservationTable{Float64}
    timers::Vector{Int}
    active_countdowns::Vector{Int}
end
get_envs(cache::PIBTCache) = cache.envs
get_solution(cache::PIBTCache) = cache.solution
get_states(cache::PIBTCache) = cache.states
get_actions(cache::PIBTCache) = cache.actions
get_ordering(cache::PIBTCache) = cache.ordering
get_undecided(cache::PIBTCache) = cache.undecided # agent ids
get_reservation_table(cache::PIBTCache) = cache.reservation_table
get_timers(cache::PIBTCache) = cache.timers
get_active_countdowns(cache::PIBTCache) = cache.active_countdowns
function get_current_priority(cache::PIBTCache,i)
    idx = findfirst(j->j==i,get_ordering(cache))
    @assert !isnothing(idx)
    return idx
end

get_active_agents(cache::PIBTCache) = findall(get_active_countdowns(cache) .<= 0)
get_inactive_agents(cache::PIBTCache) = findall(get_active_countdowns(cache) .> 0)

get_cost_model(cache::PIBTCache) = get_cost_model(cache.envs[1])

function reset_reservations!(cache::PIBTCache)
    # empty!(get_reservation_table(cache))
    reset_reservations!(get_reservation_table(cache))
    for i in get_inactive_agents(cache)
        env = get_envs(cache)[i]
        s = get_states(cache)[i]
        a = get_actions(cache)[i]
        sp = get_next_state(env,s,a)
        reserve!(cache,env,s,a,sp)
    end
end
is_reserved(cache::PIBTCache,args...) = is_reserved(get_reservation_table(cache),args...)
reserved_by(cache::PIBTCache,args...) = reserved_by(get_reservation_table(cache),args...)
# reserve!(cache::PIBTCache,args...) = reserve!(get_reservation_table(cache),args...)

function reserve!(cache::PIBTCache,args...)
    table = get_reservation_table(cache)
    valid = true
    for res in create_reservations(cache,args...)
        valid &= reserve!(table,res)
    end
    return valid
end
function create_reservations(cache::PIBTCache,args...)
    reservations = create_reservations(args...)[2:end]
end

"""
Fills `undecided` with all active agents (inactive agents have already selected their actions)
"""
function reset_undecided!(cache::PIBTCache)
    union!(get_undecided(cache),Set(get_active_agents(cache)))
end
function set_action!(cache::PIBTCache,i,a)
    get_actions(cache)[i] = a
end
function is_active(cache::PIBTCache,i)
    return get_active_countdowns(cache)[i] <= 0
end

"""
    get_conflict_index(cache,i,s,a,sp)

Returns the index of an agent that currently occupies `sp`, or -1 if there is no
such agent.
"""
function get_conflict_index(cache::PIBTCache,i,s,a,sp)
    for (k,(env,sk)) in enumerate(zip(get_envs(cache),get_states(cache)))
        if k != i && k in get_undecided(cache)
            ak = wait(env,sk)
            spk = get_next_state(env,sk,ak)
            # if states_match(sp,sk)
            if detect_state_conflict(PathNode(s,a,sp),PathNode(sk,ak,spk))
                return k
            end
        end
    end
    return -1
end

function is_consistent(cache::PIBTCache,mapf)
    for (env,s) in zip(get_envs(cache),get_states(cache))
        if !is_goal(env,s) # || is_valid(get_goal(env))
            return false
        end
    end
    return true
end

export pibt_priority_law
"""
    pibt_priority_law(solver,mapf,cache,i)

Returns a value that will determine the priority of agent i relative to other
agents. A lower value means higher priority.
"""
pibt_priority_law(solver,mapf,cache,i) = (-get_timers(cache)[i],i)

function pibt_preprocess!(solver,mapf,cache) end

function pibt_set_ordering!(solver,mapf,cache)
    get_ordering(cache) .= sortperm(
        map(i->pibt_priority_law(solver,mapf,cache,i),1:length(get_envs(cache)))
        )
    @log_info(3,verbosity(solver),"ordering: ", get_ordering(cache))
    return cache
end
function pibt_init_cache(solver,mapf,solution=get_initial_solution(mapf))
    N = num_agents(mapf)
    node = initialize_root_node(solver,mapf)
    envs = Vector{base_env_type(mapf)}(map(i->build_env(solver,mapf,node,i), 1:N))
    ordering = collect(1:N)
    undecided = Set{Int}()
    # reservation_table = Set{Int}()
    # reservation_table = PIBTReservationTable(mapf)
    reservation_table = ReservationTable{Float64}(num_states(mapf)+num_actions(mapf))
    timers = zeros(Int,N)
    end_idxs = map(get_end_index, get_paths(solution))
    active_countdowns = end_idxs .- minimum(end_idxs)
    # states = map(p->get_final_state(p), get_paths(solution))
    states = [
        get_s(p,get_end_index(p)+1-t) for (p,t) in zip(
            get_paths(solution),active_countdowns)
            ]
    # actions = map(i->wait(envs[i],states[i]), 1:N)
    actions = [
        get_a(p,get_end_index(p)+1-t) for (p,t) in zip(
            get_paths(solution),active_countdowns)
            ]
    cache = PIBTCache(
        solution,
        envs,
        states,
        actions,
        ordering,
        undecided,
        reservation_table,
        timers,
        active_countdowns,
    )
    pibt_preprocess!(solver,mapf,cache)
    pibt_set_ordering!(solver,mapf,cache)
    reset_undecided!(cache)
    reset_reservations!(cache)
    cache
end

function pibt_update_solution!(solver,solution,cache)
    for (i,(p,env,s,a)) in enumerate(zip(get_paths(solution),get_envs(cache),get_states(cache),get_actions(cache)))
        if !is_active(cache,i)
            continue
        end
        sp = get_next_state(env,s,a)
        add_to_path!(p,env,s,a,sp)
        set_path_cost!(solution,get_cost(p),i)
    end
    set_cost!(solution, aggregate_costs(
        get_cost_model(cache),
        get_path_costs(solution)
        ))
    solution
end
function pibt_update_env!(solver,mapf,cache,i)
    node = initialize_root_node(solver,mapf)
    # Trying to set a dummy task for the goal (serves as a marker that this agent's task is complete)
    # But still need to set agent_id = i
    env = build_env(solver,mapf,node,-1)
    get_envs(cache)[i] = base_env_type(mapf)(env,agent_idx=i)
end
function pibt_update_envs!(solver,mapf,cache)
    for (i,(s,env)) in enumerate(zip(get_states(cache),get_envs(cache)))
        if !is_active(cache,i)
            continue
        end
        if is_goal(env,s) && is_valid(get_goal(env))
            pibt_update_env!(solver,mapf,cache,i)
            get_timers(cache)[i] = 0
        end
    end
end
function pibt_update_cache!(solver,mapf,cache)
    pibt_update_solution!(solver,get_solution(cache),cache)
    get_timers(cache) .+= 1
    get_active_countdowns(cache) .= max.(0, get_active_countdowns(cache) .- 1)
    for (i,p) in enumerate(get_paths(get_solution(cache)))
        t = get_end_index(p)+1-get_active_countdowns(cache)[i]
        get_states(cache)[i] = get_s(p,t)
        get_actions(cache)[i] = get_a(p,t)
    end
    reset_undecided!(cache)
    reset_reservations!(cache)
    pibt_update_envs!(solver,mapf,cache)
    if any(get_timers(cache) .== 0)
        pibt_set_ordering!(solver,mapf,cache)
    end
    return cache
end
function pibt_next_agent_id(solver,cache)
    for i in get_ordering(cache)
        if i in get_undecided(cache)
            return i
        end
    end
    return -1
end

export pibt_step!

"""
    pibt_step!(solver,mapf,i,j=-1)

i is the id of the higher priority agent, j is the index of the lower priority
agent.
"""
function pibt_step!(solver,mapf,cache,i=pibt_next_agent_id(solver,cache),j=-1,PRIORITY=-1)
    env = get_envs(cache)[i]
    s = get_states(cache)[i]
    sj = get(get_states(cache), j, state_type(mapf)())
    a_list = sorted_actions(env,s) # NOTE does NOT need to exclude wait()
    @log_info(3,verbosity(solver),"PIBT: pibt_step!( ... i = ",i,", j = ",j," )")
    @log_info(3,verbosity(solver),"  s        = ",string(s))
    @log_info(3,verbosity(solver),"  actions  = ",map(string,a_list))
    @log_info(3,verbosity(solver),"  env.goal = ",string(get_goal(env)))
    setdiff!(cache.undecided,i) # NOTE does this need to be uncommented after all?
    while ~isempty(a_list)
        a = a_list[1]
        sp = get_next_state(env,s,a)
        if !is_reserved(cache,env,s,a,sp) && !states_match(sp,sj) # Problem: Does not filter out self reservations
            @log_info(3,verbosity(solver),"  agent $i reserve!( ... a = ",string(a),", sp = ",string(sp)," )")
            reserve!(cache,env,s,a,sp)
            k = get_conflict_index(cache,i,s,a,sp)
            if k != -1
                @log_info(3,verbosity(solver),"  agent $i get_conflict_index( i = ",i,", sp = ",string(sp)," ) : ",k)
                if pibt_step!(solver,mapf,cache,k,i)
                    set_action!(cache,i,a)
                    # setdiff!(cache.undecided,i)
                    return true
                else
                    deleteat!(a_list,1)
                    # break # <--- NOTE is this the problem? What if we don't break here?
                end
            else
                set_action!(cache,i,a)
                # setdiff!(cache.undecided,i)
                return true
            end
        else
            @log_info(3,verbosity(solver),"  agent $i illegal action ",string(a),
                ". Reserved by ",reserved_by(cache,env,s,a,sp))
            deleteat!(a_list,1)
        end
    end
    set_action!(cache,i,wait(env,s))
    return false
end

export pibt!

function pibt!(solver, mapf)
    cache = pibt_init_cache(solver,mapf)
    @log_info(0,verbosity(solver),"PIBT: Entering...")
    while !is_consistent(cache,mapf)
        try
            increment_iteration_count!(solver)
            enforce_iteration_limit!(solver)
            enforce_time_limit!(solver)
        catch e
            if isa(e,SolverException)
                handle_solver_exception(solver,e)
                break
            else
                rethrow(e)
            end
        end
        @log_info(1,verbosity(solver),"PIBT iterations = ",iterations(solver))
        while !isempty(cache.undecided)
            if ~pibt_step!(solver,mapf,cache)
                return get_solution(cache), false
            end
        end
        # update cache
        pibt_update_cache!(solver,mapf,cache)
        @log_info(3,verbosity(solver),"solution: ",convert_to_vertex_lists(get_solution(cache)))
    end
    if solver.partial
        return get_solution(cache), true
    end
    return get_solution(cache), is_consistent(cache,mapf)
end

function solve!(solver::PIBTPlanner,mapf)
    solution, valid = pibt!(solver,mapf)
    if valid
        @log_info(0,verbosity(solver),"SUCCESS! Valid solution returned by PIBT")
        set_best_cost!(solver, get_cost(solution))
    else
        @log_info(0,verbosity(solver),"FAILED! Invalid solution returned by PIBT")
        set_cost!(solution,typemax(cost_type(mapf)))
    end
    return solution, best_cost(solver)
end
