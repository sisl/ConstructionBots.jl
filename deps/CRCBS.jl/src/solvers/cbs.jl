export CBSSolver

"""
    CBSSolver

Path planner that employs Conflict-Based Search.
"""
@with_kw struct CBSSolver{L,C} <: AbstractCBSSolver
    low_level_planner::L    = VanillaAStar()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}(
        iteration_limit=1000,
    )
end
CBSSolver(planner) = CBSSolver(low_level_planner=planner)
# CBSSolver(mapf::AbstractMAPF) = CBSSolver(low_level_planner=AStar{cost_type(mapf)}())
for op in [:set_deadline!,:set_runtime_limit!,:set_verbosity!]
    eval(quote
        $op(solver::CBSSolver,args...) = begin
            $op(get_logger(solver),args...)
            $op(solver.low_level_planner,args...)
        end
    end)
end

################################################################################
############################# CBS Logger Interface #############################
################################################################################
function enter_cbs!(solver)
    reset_solver!(solver)
end
function logger_cbs_add_constraint!(solver,new_node,constraint) end
function logger_dequeue_cbs_node!(solver,mapf,node)
    @log_info(1,verbosity(solver),
        "CBS: iter ",iterations(solver)," - node ",node.trace,
        " - Current paths: \n",
        sprint_indexed_list_array(convert_to_vertex_lists(node.solution);leftaligned=true),
        # sprint_route_plan(node.solution;leftaligned=true),
        )
    @log_info(2,verbosity(solver),"CBS: constraints in node ",node.trace,": \n",
        map(c->string("\t",string(c),"\n"),sorted_state_constraints(mapf,node))...,
        map(c->string("\t",string(c),"\n"),sorted_action_constraints(mapf,node))...,
        )
    # if verbosity(solver) >= 1
    #     println("CBS: iter ",iterations(solver)," - node ",node.trace," - Current paths: ")
    #     for (i,path) in enumerate(get_paths(node.solution))
    #         println("\t",i,": ",convert_to_vertex_lists(path))
    #     end
    # end
    enforce_iteration_limit!(solver)
end
function logger_exit_cbs_optimal!(solver,node)
    set_best_cost!(solver,get_cost(node))
    @log_info(0,verbosity(solver),"Optimal solution found by CBS! Cost = $(get_cost(node))")
    # if verbosity(solver) >= 0
    #     println("Optimal Solution Found! Cost = $(get_cost(node))")
    # end
end
function logger_cbs_add_constraint!(solver,node,constraint,mapf)
    increment_iteration_count!(solver)
    enforce_time_limit!(solver)
    enforce_iteration_limit!(solver)
    @log_info(1,verbosity(solver),"CBS: adding constraint to node ",node.trace,": ",string(constraint))
    # @log_info(2,verbosity(solver),"CBS: constraints in node ",node.trace,": \n",
    #     map(c->string("\t",string(c),"\n"),sorted_state_constraints(mapf,node))...,
    #     map(c->string("\t",string(c),"\n"),sorted_action_constraints(mapf,node))...,
    #     )
    # if verbosity(solver) == 1
    #     println("CBS: adding constraint ",string(constraint))
    # elseif verbosity(solver) >= 2
    #     # println("CBS: adding constraint:")
    #     # println("\t",string(constraint))
    #     println("CBS: constraints in node:")
    #     for i in 1:num_agents(mapf)
    #         for constraint in sorted_state_constraints(mapf,node,i)
    #             println("\t",string(constraint))
    #         end
    #         for constraint in sorted_action_constraints(mapf,node,i)
    #             println("\t",string(constraint))
    #         end
    #     end
    # end
end

################################################################################
################################ CBS Interface #################################
################################################################################
export low_level_search!

"""
    low_level_search!(
        solver,
        mapf::AbstractMAPF,
        node::ConstraintTreeNode,
        idxs=collect(1:num_agents(mapf)),
        path_finder=a_star!)

    Returns a low level solution for a MAPF with constraints. The heuristic
    function for cost-to-go is user-defined and environment-specific
"""
function low_level_search!(
    solver, mapf::M, node::N, idxs=collect(1:num_agents(mapf));
    path_finder=a_star!
    ) where {M<:AbstractMAPF,N<:ConstraintTreeNode}
    # Only compute a path for the indices specified by idxs
    for i in idxs
        env = build_env(solver, mapf, node, i)
        reset_solver!(low_level(solver))
        path, cost = path_finder(low_level(solver), env, get_start(mapf,env,i))
        set_solution_path!(node.solution, path, i)
        set_path_cost!(node.solution, cost, i)
    end
    set_cost!(node, aggregate_costs(get_cost_model(mapf.env),get_path_costs(node.solution)))
    return is_consistent(node.solution,mapf)
end
# low_level_search!(solver::CBSSolver,args...) = low_level_search!(low_level(solver),args...)
build_env(solver, mapf, node, i) = build_env(mapf, node, i)

"""
    get_agent_idxs(solver,node,mapf,constraint)

Part of CBS interface. Defaults to return the index of a single agent affected
by a constraint. Can be overridden to return the index of e.g., a "meta-agent"
(group of agents).
"""
function get_agent_idxs(solver,node,mapf,constraint)
    [get_agent_id(constraint)]
end

"""
    cbs_bypass!(solver,mapf,node,conflict,priority_queue)

Part of CBS interface. Defaults to false, but can be overridden to modify the
priority_queue and/or bypass the branching step of CBS
"""
cbs_bypass!(solver,mapf,node,priority_queue) = false

"""
    cbs_update_conflict_table!(solver,mapf,node,constraint)

Allows for flexible conflict-updating dispatch. This function is called within
    within the default `cbs_branch!()` method.
"""
function cbs_update_conflict_table!(solver,mapf,node,constraint)
    detect_conflicts!(node,[get_agent_id(constraint)]) # update conflicts related to this agent
end

"""
    cbs_branch!(solver,mapf,node,conflict,priority_queue)

Part of CBS interface. Defaults to splitting on the conflict and adding two
nodes to the priority_queue, where each of the child nodes has one of the new
complementary constraints.
"""
function cbs_branch!(solver,mapf,node,conflict,priority_queue)
    constraints = generate_constraints_from_conflict(conflict)
    for (i,constraint) in enumerate(constraints)
        new_node = initialize_child_search_node(solver,mapf,node)
        set_trace!(new_node,node,i)
        logger_cbs_add_constraint!(solver,new_node,constraint,mapf)
        if add_constraint!(mapf,new_node,constraint)
            consistent_flag = low_level_search!(solver, mapf, new_node,[get_agent_id(constraint)])
            if consistent_flag
                cbs_update_conflict_table!(solver,mapf,new_node,constraint)
                enqueue!(priority_queue, new_node => get_cost(new_node))
            end
        end
    end
    priority_queue
end

"""
    Conflict-Based Search

    Sharon et al 2012
    https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239
"""
function cbs!(solver,mapf)
    enter_cbs!(solver)
    node = initialize_root_node(solver,mapf)
    priority_queue = PriorityQueue{typeof(node),cost_type(node)}()
    consistent_flag = low_level_search!(solver,mapf,node)
    if consistent_flag
        detect_conflicts!(node)
        enqueue!(priority_queue, node => get_cost(node))
    end

    try
        while ~isempty(priority_queue)
            # node = cbs_dequeue_and_preprocess!(solver,priority_queue,mapf)
            node = dequeue!(priority_queue)
            set_lower_bound!(solver,get_cost(node))
            logger_dequeue_cbs_node!(solver,mapf,node)
            # check for conflicts
            conflict = get_next_conflict(node.conflict_table)
            if !is_valid(conflict)
                logger_exit_cbs_optimal!(solver,node)
                return node.solution, get_cost(node)
            elseif ~cbs_bypass!(solver,mapf,node,priority_queue)
                cbs_branch!(solver,mapf,node,conflict,priority_queue)
            end
        end
    catch e
        isa(e, SolverException) ? handle_solver_exception(solver,e) : rethrow(e)
    end
    @log_info(-1,verbosity(solver),"CBS: No Solution Found. Returning default solution")
    return default_solution(mapf)
end

function solve!(solver::CBSSolver, mapf::M where {M<:AbstractMAPF}, path_finder=a_star!;verbose=false)
    cbs!(solver,mapf)
end
