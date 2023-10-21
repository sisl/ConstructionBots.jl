export AbstractAStarPlanner
abstract type AbstractAStarPlanner end

export
    logger_enter_a_star!,
    logger_exit_a_star!,
    logger_step_a_star!,
    logger_find_constraint_a_star!,
    logger_enqueue_a_star!

check_termination_criteria(solver, env, cost_so_far, s)  =  iterations(solver) > iteration_limit(solver)
function logger_enter_a_star!(solver, env, base_path)
    @log_info(2,verbosity(solver),"A*: entering...")
    # @log_info(3,verbosity(solver),"A*: env: ",sprint(show, env))
    @log_info(3,verbosity(solver),"A*: start state: ",string(get_final_state(base_path)))
    @assert(iterations(solver) == 0, "A*: ERROR: iterations = $(iterations(solver)) at entry")
end
function logger_exit_a_star!(solver, path, cost, status)
    # empty!(solver.search_history)
    if status == false
        @log_info(0,verbosity(solver),"A*: failed to find feasible path. Returning path of cost $cost")
        @log_info(0,verbosity(solver),"A*:            timed_out = $(time_out_status(solver))",
            " -- run time limit = $(runtime_limit(solver))")
        @log_info(0,verbosity(solver),"A*: iterations maxed out = $(iteration_max_out_status(solver))",
            " -- iterations = $(iterations(solver))")
    else
        @log_info(2,verbosity(solver),"A*: returning optimal path with cost $cost")
        @log_info(4,verbosity(solver),"A*: path = \n",string("\t",summary(get_initial_state(path)),"\n"),[string("\t",summary(n.sp),"\n") for n in path.path_nodes]...)
    end
end
function logger_step_a_star!(solver, env, base_path, s, q_cost, cost_so_far)
    increment_iteration_count!(solver)
    @log_info(4,verbosity(solver),"A*: iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost, true_cost = $cost_so_far")
end
function logger_enqueue_a_star!(solver, env, s, a, sp, h_cost)
    @log_info(5,verbosity(solver),"A*: exploring $(string(s)) -- $(string(sp)), h_cost = $h_cost")
end
function logger_find_constraint_a_star!(logger,env,s,a,sp)
    @log_info(1,verbosity(logger),"A*: sequence ",string(s),", ",string(a),", ",string(sp),
        " violates a constraint")
end

export AStar

"""
    AStar

A* Path Planner.
Fields:
- logger
- replan : if true, planner will replan with an empty conflict table following
    timeout.
"""
@with_kw struct AStar{C} <: AbstractAStarPlanner
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
# AStar() = AStar{Float64}()
export VanillaAStar
VanillaAStar() = AStar{Float64}()

export
    a_star_impl!,
    a_star!

check_termination_criteria(solver,env,cost_so_far,path,s) = check_termination_criteria(env,cost_so_far,path,s)

"""
    reconstruct path by working backward from the terminal state
"""
function reconstruct_path!(path,predecessor_map,s,cost)
    node_sequence = Vector{node_type(path)}()
    while haskey(predecessor_map, s)
        path_node = predecessor_map[s]
        push!(node_sequence, path_node)
        s = get_s(path_node)
    end
    reverse!(node_sequence)
    for n in node_sequence
        push!(path,n)
    end
    set_cost!(path,cost)
    return path
end

"""
    The internal loop of the A* algorithm.

    # g(n) = cost of the path from the start node to n,
    # h(n) = heuristic estimate of cost from n to goal
    # f(n) = g(n) + h(n)
"""
function a_star_impl!(solver, env::E, base_path, frontier, explored) where {E <: AbstractLowLevelEnv}
    logger_enter_a_star!(solver, env, base_path)
    cost_map = Dict{state_type(env),cost_type(env)}() # TODO: get some speedup by converting states to indices (then use a vector instead of a dict)
    predecessor_map = Dict{state_type(env),node_type(env)}() # TODO: get some speedup by converting states to indices (then use a vector instead of a dict)
    default_cost = get_infeasible_cost(env)
    opt_status = false
    while !isempty(frontier)
        (cost_so_far, s), q_cost = dequeue_pair!(frontier)
        logger_step_a_star!(solver,env,base_path,s,q_cost,cost_so_far)
        if is_goal(env,s)
            opt_status = true
            r_path = reconstruct_path!(base_path,predecessor_map,s,cost_so_far)
            logger_exit_a_star!(solver,r_path,cost_so_far,opt_status)
            return r_path, cost_so_far
        elseif check_termination_criteria(solver,env,cost_so_far,s)
            break
        end

        for a in get_possible_actions(env,s)
            sp = get_next_state(env,s,a)
            if violates_constraints(env,s,a,sp) # Skip node if it violates any of the constraints
                logger_find_constraint_a_star!(solver,env,s,a,sp)
                continue
            end
            if !(sp in explored)
                new_cost = accumulate_cost(env, cost_so_far, get_transition_cost(env,s,a,sp))
                if new_cost < get(cost_map, sp, default_cost)
                    cost_map[sp] = new_cost
                    predecessor_map[sp] = PathNode(s, a, sp) # track predecessor
                    h_cost = compute_heuristic_cost(env, new_cost, sp)
                    logger_enqueue_a_star!(solver,env,s,a,sp,h_cost)
                    enqueue!(frontier, (new_cost, sp) => h_cost)
                end
            end
        end
        push!(explored,s)
    end
    path = path_type(env)(cost=get_infeasible_cost(env))
    cost = path.cost
    logger_exit_a_star!(solver,path,cost,opt_status)
    return path, cost
end


"""
a_star!(env,start_state)

A generic implementation of the [A* search algorithm](http://en.wikipedia.org/wiki/A%2A_search_algorithm)
that operates on an Environment and initial state.

args:
- `env::E <: AbstractLowLevelEnv`
- `start_state`

The following methods must be implemented:
- is_goal(env::E,s::S)
- check_termination_criteria(env::E,cost::C,path::Path{S,A,C},state::S)
- get_possible_actions(env::E,s::S)
- get_next_state(env::E,s::S,a::A,sp::S)
- get_transition_cost(env::E,s::S,a::A)
- violates_constraints(env::E,s::S,path::Path{S,A,C})
"""
function a_star!(solver, env::E,path::P,initial_cost=get_cost(path)) where {E<:AbstractLowLevelEnv,P<:AbstractPath}
    frontier = PriorityQueue{Tuple{cost_type(env), state_type(env)}, cost_type(env)}()
    enqueue!(frontier, (initial_cost, get_final_state(path))=>initial_cost)
    explored = Set{state_type(env)}()
    a_star_impl!(solver,env,path,frontier,explored)
end
function a_star!(env::E,path::P,initial_cost=get_cost(path)) where {E<:AbstractLowLevelEnv,P<:AbstractPath}
    a_star!(nothing,env,path,initial_cost)
end
function a_star!(solver, env::E,start_state,initial_cost=get_initial_cost(env)) where {E<:AbstractLowLevelEnv}
    path = path_type(env)(s0=start_state,cost=initial_cost)
    a_star!(solver,env,path,initial_cost)
end
function a_star!(env::E,start_state,initial_cost=get_initial_cost(env)) where {E<:AbstractLowLevelEnv}
    path = path_type(env)(s0=start_state,cost=initial_cost)
    a_star!(env,path,initial_cost)
end
