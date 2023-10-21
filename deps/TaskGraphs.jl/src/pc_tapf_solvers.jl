# module Solvers
#
# using Parameters
# using MathOptInterface, JuMP
# using GraphUtils
# using DataStructures
#
# using CRCBS
# using ..TaskGraphs

"""
    AbstractPCTAPFSolver

Abstract type of which all PC-TAPF solvers must be concrete subtypes. All
concrete solvers must implement the following interface for solving PC-TAPF
problems:
- `solution, cost = solve!(solver,problem_def)`
- `check_runtime(solver)` should trigger an interrupt + early return if the
    allowable runtime has been exceeded

Also, we need a good Logger type for keeping track of thing like runtime,
iterations, optimality gap (including upper and lower bound), etc.
"""
abstract type AbstractPCTAPFSolver end

abstract type SearchTrait end
struct Prioritized <: SearchTrait end
struct NonPrioritized <: SearchTrait end
function search_trait end

primary_cost(solver,cost) = cost[1]
primary_cost_type(solver) = Float64

export AStarSC

"""
    AStarSC

Low-level path planner that employs Slack-and-Collision-Aware A*.
Fields:
- logger
- replan : if true, planner will replan with an empty conflict table following
    timeout.
"""
@with_kw struct AStarSC{C} <: AbstractAStarPlanner
    logger::SolverLogger{C} = SolverLogger{C}(iteration_limit=1000)
    replan::Bool            = true # false
    use_slack_cost          = true
    use_conflict_cost       = true
end
search_trait(solver::AStarSC) = NonPrioritized()
function CRCBS.logger_step_a_star!(solver::AStarSC, env, base_path, s, q_cost)
    increment_iteration_count!(solver)
    # if solver.DEBUG
    #     push!(solver.search_history, s)
    # end
    if check_iterations(solver)
        # if solver.DEBUG
        #     # Dump env to JLD2 environment
        #     filename = joinpath(DEBUG_PATH,string("A_star_dump_",get_debug_file_id(),".jld2"))
        #     mkpath(DEBUG_PATH)
        #     @log_info(-1,verbosity(solver),"Dumping A* env to $filename")
        #     agent_id = env.agent_idx
        #     history = map(s->(s.vtx,s.t),solver.search_history)
        #     start   = (get_final_state(base_path).vtx,get_final_state(base_path).t)
        #     goal    = (env.goal.vtx,env.goal.t)
        #     idx = isa(solver.AStarSC) ? 2 : 1
        #     paths   = get_cost_model(env).cost_models[idx].model.table.paths
        #     state_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.state_constraints))
        #     action_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.action_constraints))
        #     @save filename agent_id history start goal paths state_constraints action_constraints
        #     # @assert false "making a bogus assertion to hack my way out of this block"
        # end
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
    @log_info(2,verbosity(solver),"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end

export DefaultAStarSC
const DefaultAStarSC = AStarSC{NTuple{5,Float64}}

# export
#     PrioritizedAStarSC

# """
#     PrioritizedAStarSC

# Low-level proritized path planner that employs Slack-and-Collision-Aware A*.
# """
# @with_kw struct PrioritizedAStarSC{C}
#     logger::SolverLogger{C} = SolverLogger{C}()
#     replan::Bool            = false
# end
# search_trait(solver::PrioritizedAStarSC) = Prioritized()
# primary_cost(::PrioritizedAStarSC,cost::NTuple{5,Float64}) = cost[2]

"""
    construct_heuristic_model(solver,env_graph;kwargs...)

Construct the heuristic model to be used by solver.
"""
function construct_heuristic_model(trait::NonPrioritized,solver,env_graph,
        ph = EnvDistanceHeuristic(), # coupled with slack cost
        )
    construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
end
function construct_heuristic_model(solver, args...)
    construct_heuristic_model(search_trait(solver),solver,args...)
end

"""
    construct_cost_model(solver::AStarSC, args...;kwargs...)

Defines the cost model used by Slack- and Collision-aware A*.
This particular setting of cost model is crucial for good performance of A_star,
because it encourages depth first search. If we were to replace terms 3-5 with
SumOfTravelTime(), we would get worst-case exponentially slow breadth-first
search!
"""
function construct_cost_model(trait::NonPrioritized,
        solver, sched, cache, problem_spec, env_graph, primary_objective=SumOfMakeSpans();
        use_conflict_cost::Bool=solver.use_conflict_cost,
        use_slack_cost::Bool=solver.use_slack_cost,
        extra_T::Int=2000)
    N = length(get_robot_ICs(sched))
    @assert N > 0 "num_robots should be > 0. We need at least one robot!"
    if use_conflict_cost
        conflict_cost = HardConflictCost(env_graph,makespan(sched)+extra_T, N)
    else
        # @log_info(1,verbosity(solver),"Forgoing conflict cost for node")
        # conflict_cost = NullCost()
        conflict_cost = FullCostModel(sum,NullCost())
    end
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(sched,cache),
        conflict_cost,
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()), # SumOfTravelTime(),
        FullCostModel(sum,TransformCostModel(c->-1*c,TravelTime()))
    )
    heuristic_model = construct_heuristic_model(trait,solver,env_graph)
    if !use_slack_cost
        # reorder so that travel distance is prioritized ahead of conflicts
        ordering = [1,3,2,4,5]
        cost_model = construct_composite_cost_model(
            cost_model.cost_models[ordering]...
            )
        heuristic_model = construct_composite_heuristic(
            heuristic_model.cost_models[ordering]...
            )
    end
    cost_model, heuristic_model
end
function construct_cost_model(trait::SearchTrait,
        solver, env::SearchEnv,
        primary_objective=get_problem_spec(env).cost_function
        ;
        kwargs...)
    construct_cost_model(trait,solver,
        get_schedule(env), get_cache(env), get_problem_spec(env), get_graph(env),
        primary_objective;kwargs...)
end
function construct_cost_model(solver, args...;kwargs...)
    construct_cost_model(search_trait(solver),solver,args...;kwargs...)
end

export
    update_route_plan!

"""
    update_route_plan!()
"""
function update_route_plan!(solver,pc_mapf::AbstractPC_MAPF,env,v,path,cost,node,
    agent_id=get_id(get_default_robot_id(node)))
    set_solution_path!(env, path, agent_id)
    set_path_cost!(env, cost, agent_id)
    update_env!(solver,env,v,path,agent_id)
    set_cost!(env,aggregate_costs(get_cost_model(env),get_path_costs(env)))
    return env
end
function update_route_plan!(solver,pc_mapf::C_PC_MAPF,env,v,meta_path,meta_cost,schedule_node)
    # add to solution
    paths = MetaAgentCBS.split_path(meta_path)
    # @show length(paths)
    # @show length(meta_env.envs)
    # @show length(meta_cost.independent_costs)
    # @show length(schedule_node.instructions)
    for (new_path, sub_node) in zip(paths, sub_nodes(schedule_node))
        agent_id = get_id(get_robot_id(sub_node))
        path = new_path
        update_route_plan!(solver,PC_MAPF(get_env(pc_mapf)),env,v,path,get_cost(path),sub_node,agent_id)
        @log_info(3,verbosity(solver),"agent_id = ", agent_id)
        @log_info(3,verbosity(solver),string("agent_path = ", convert_to_vertex_lists(path)))
        @log_info(3,verbosity(solver),string("cost = ", get_cost(new_path)))
    end
    return env
end

export
    plan_path!

path_finder(solver::AStarSC,args...) = CRCBS.a_star!(solver,args...)

"""
    plan_path!

    Computes next path specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function plan_path!(solver::AStarSC, pc_mapf::AbstractPC_MAPF, env::SearchEnv, node::N,
        schedule_node::T, v::Int;
        backtrack::Bool=false,
        kwargs...) where {N<:ConstraintTreeNode,T}

    cache = get_cache(env)
    reset_solver!(solver)
    if backtrack
        # Construct cost model without collision cost (Currently, this does not work)
        cost_model, _ = construct_cost_model(solver, env, use_conflict_cost=false)
        cbs_env = build_env(solver, pc_mapf, env, node, VtxID(v);cost_model=cost_model)
    else
        cbs_env = build_env(solver, pc_mapf, env, node, VtxID(v)) 
    end
    base_path = get_base_path(solver,env,cbs_env)
    ### PATH PLANNING ###
    # solver.DEBUG ? validate(base_path,v) : nothing
    path, cost = path_finder(solver, cbs_env, base_path)
    @assert get_cost(path) == cost
    if cost == get_infeasible_cost(cbs_env)
        if solver.replan == true
            @log_info(-1,verbosity(solver),"A*: replanning without conflict cost", string(schedule_node))
            reset_solver!(solver)
            cost_model, _ = construct_cost_model(solver, env;
                use_conflict_cost=false,)
            cbs_env = build_env(solver, pc_mapf, env, node, VtxID(v);cost_model=cost_model)
            base_path = get_base_path(solver,env,cbs_env)
            path, cost = path_finder(solver, cbs_env, base_path)
        end
        if cost == get_infeasible_cost(cbs_env)
            @log_info(-1,verbosity(solver),"A*: returned infeasible path for node ", string(schedule_node))
            return false
        end
    end
    # solver.DEBUG ? validate(path,v,cbs_env) : nothing
    #####################
    @log_info(2,verbosity(solver),string("A* iterations = ",iterations(solver)))
    # Make sure every robot sticks around for the entire time horizon
    if is_terminal_node(get_graph(get_schedule(env)),v)
        @log_info(2,verbosity(solver),"ISPS: length(path) = ",length(path),
        ". Extending terminal node ", string(schedule_node),
        " to ",makespan(get_schedule(env)))
        # extend_path!(cbs_env,path,maximum(cache.tF))
        extend_path!(cbs_env,path,makespan(get_schedule(env)))
        @log_info(2,verbosity(solver),"ISPS: length(path) = ",length(path),
        ". maximum(cache.tF) = ",makespan(get_schedule(env)))
        # solver.DEBUG ? validate(path,v) : nothing
    end
    # add to solution
    update_route_plan!(solver,pc_mapf,env,v,path,cost,schedule_node)
    @log_info(2,verbosity(solver),"ISPS: after update_route_plan! maximum(cache.tF) = ",
        makespan(get_schedule(env)))
    if get_cost(node) >= best_cost(solver)
        @log_info(0,verbosity(solver),"ISPS: get_cost(node) (",get_cost(node),
            ") >= best_cost(solver)",best_cost(solver)," ... Exiting early")
        return false
    end

    return true
end

export
    ISPS

"""
    ISPS

Path planner that employs Incremental Slack-Prioritized Search.
"""
@with_kw struct ISPS{L,C} <: BiLevelPlanner
    low_level_planner::L    = DefaultAStarSC()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}(
        iteration_limit = 2
    )
    tighten_paths::Bool     = true
    # backtracking for completeness
    backtrack::Bool         = false
    backtracking_activated  = Toggle(false)
    backtrack_list::Dict{AbstractID,Bool} = Dict{AbstractID,Bool}()
    # slack priority toggle (for ablation studies)
    random_prioritization::Bool = false
end
ISPS(planner) = ISPS(low_level_planner=planner)
construct_cost_model(solver::ISPS,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::ISPS) = search_trait(low_level(solver))
primary_cost(solver::ISPS,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::ISPS) = primary_cost_type(low_level(solver))
function CRCBS.set_best_cost!(solver::ISPS,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function CRCBS.hard_reset_solver!(solver::ISPS)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end
for op in [:set_deadline!,:set_runtime_limit!,:set_verbosity!]
    eval(quote
        CRCBS.$op(solver::ISPS,args...) = begin
            $op(get_logger(solver),args...)
            $op(low_level(solver),args...)
        end
    end)
end
function do_backtrack(solver::ISPS,sched,id) 
    flag = get_toggle_status(solver.backtracking_activated) && get(solver.backtrack_list,id,false)
    if flag
        @log_info(1,verbosity(solver),"do_backtrack = true for $(summary(get_node(sched,id)))")
    end
    return flag
end
function needs_backtrack(solver::ISPS,sched,id) 
    flag = solver.backtrack && get(solver.backtrack_list,id,false)
    if flag
        @log_info(1,verbosity(solver),"needs_backtrack = true for $(summary(get_node(sched,id)))")
    end
    return flag
end
"""
    update_backtrack_list!(solver::ISPS,sched,id,val=false)

Add to backtrack_list.
"""
function update_backtrack_list!(solver::ISPS,sched,id,val=true)
    @log_info(1,verbosity(solver),"adding to backtrack list: $(summary(get_node(sched,id)))")
    solver.backtrack_list[id] = val
end
function activate_backtracking!(solver::ISPS)
    @log_info(1,verbosity(solver),"activating backtracking ...")
    if isempty(solver.backtrack_list) || solver.backtrack == false
        @log_info(1,verbosity(solver),"backtracking not activated")
        return false
    end
    set_toggle_status!(solver.backtracking_activated,true)
    @log_info(1,verbosity(solver),"backtracking activated")
    return get_toggle_status(solver.backtracking_activated)
end
function clear_backtrack_list!(solver::ISPS)
    set_toggle_status!(solver.backtracking_activated,false)
    empty!(solver.backtrack_list)
end
"""
    isps_queue_cost(solver::ISPS,sched::OperatingSchedule,v::Int)

Allow prioritization of nodes flagged for backtracking.
"""
function isps_queue_cost(solver::ISPS,sched::OperatingSchedule,v::Int)
    path_spec = get_path_spec(sched,v)
    if do_backtrack(solver,sched,get_vtx_id(sched,v))
        override = 0
    else
        override = Int(path_spec.plan_path)
    end
    if solver.random_prioritization
        slack_cost = rand()
    else
        slack_cost = minimum(get_slack(sched,v))
    end
    return (override, slack_cost)
end

update_isps_queue!(solver::ISPS,env::SearchEnv) = update_isps_queue!(solver,get_cache(env),get_schedule(env))
function update_isps_queue!(solver::ISPS,cache,sched)
    for v in cache.active_set
        cache.node_queue[v] = isps_queue_cost(solver,sched,v)
    end
end

export
    plan_next_path!,
    compute_route_plan!

"""
    `plan_next_path`
"""
function plan_next_path!(solver::ISPS, pc_mapf::AbstractPC_MAPF, env::SearchEnv, node::N
        ) where {N<:ConstraintTreeNode}

    valid_flag = true
    if ~isempty(get_cache(env).node_queue)
        v,priority = dequeue_pair!(get_cache(env).node_queue)
        n_id = get_vtx_id(get_schedule(env),v)
        schedule_node = get_node_from_id(get_schedule(env),n_id)
        if get_path_spec(get_schedule(env), v).plan_path == true
            try
                @log_info(2,verbosity(solver),sprint(show,env))
                @log_info(2,verbosity(solver),
                """
                ISPS:
                    schedule_node: $(string(schedule_node))
                    get_tF(sched,v): $(get_tF(env,v))
                    maximum(get_cache(env).tF): $(makespan(get_schedule(env)))
                """)
                dt = get_duration(get_schedule(env),n_id)
                valid_flag = plan_path!(low_level(solver),pc_mapf,env,node,schedule_node,v;
                    backtrack=do_backtrack(solver,get_schedule(env),n_id),
                )
                if solver.backtrack && get_duration(get_schedule(env),n_id) > dt
                    update_backtrack_list!(solver,get_schedule(env),n_id)
                end
                # update priority queue using isps_queue_cost(solver,...)
                update_isps_queue!(solver,env)
                @log_info(2,verbosity(solver),"""
                ISPS:
                    routes:
                """,
                sprint_indexed_list_array(
                    convert_to_vertex_lists(get_route_plan(env));leftaligned=true),
                """
                    cache.tF: $(get_tF(env,v))
                    get_tF(sched,v): $(get_tF(env,v))
                """)
            catch e
                if isa(e, SolverException)
                    handle_solver_exception(solver,e)
                    @log_info(-1,verbosity(solver),"Exiting ISPS with failed status")
                    valid_flag = false
                    # return valid_flag
                    return false
                else
                    rethrow(e)
                end
            end
        else
            update_planning_cache!(solver,env,v,-1)
        end
    end
    return valid_flag
end

"""
    compute_route_plan!

Computes all paths specified by the project schedule and updates the solution
in the ConstraintTreeNode::node accordingly.
"""
function compute_route_plan!(solver::ISPS, pc_mapf::AbstractPC_MAPF, node::N, env::SearchEnv=node.solution;
        kwargs...
        ) where {N<:ConstraintTreeNode}

    while length(get_cache(env).node_queue) > 0
        status = plan_next_path!(solver,pc_mapf,env,node)
        status ? nothing : return false
        status = tighten_gaps!(solver,pc_mapf,env,node)
        status ? nothing : return false
        enforce_time_limit!(solver)
    end
    return true
end

function CRCBS.low_level_search!(solver::ISPS, pc_mapf::AbstractPC_MAPF,
        node::N=initialize_root_node(solver,pc_mapf),
        idxs::Vector{Int}=Vector{Int}()
    ) where {N<:ConstraintTreeNode}

    reset_solver!(solver) # problematic because the lower_bound from assignment is not stored?
    search_env = node.solution
    conflict_table = deepcopy(node.conflict_table)
    update_isps_queue!(solver,search_env)
    if solver.backtrack
        clear_backtrack_list!(solver)
    end
    valid_flag = true
    for i in 1:iteration_limit(solver)
        increment_iteration_count!(solver)
        # reset solution
        reset_schedule_times!(get_schedule(search_env),get_schedule(get_env(pc_mapf)))
        reset_cache!(get_cache(search_env),get_schedule(search_env),)
        reset_route_plan!(node,get_route_plan(get_env(pc_mapf)))
        valid_flag = compute_route_plan!(solver, pc_mapf, node)
        if valid_flag == false
            @log_info(0,verbosity(solver),"ISPS: failed on iteration $i.")
            break
        else
            conflict_table = detect_conflicts(node.solution)
            if count_conflicts(conflict_table) == 0 # if route plan is valid, break
                @log_info(2,verbosity(solver),"ISPS: returning valid solution on iteration $i.")
                break
            end
        end
    end
    # Needs to happen in while loop
    if solver.backtrack && valid_flag && count_conflicts(conflict_table) > 0
        # How to identify need for backtracking (i.e., with optimality_gap)?
        need_backtrack = false
        sched = get_schedule(search_env)
        base_sched = get_schedule(get_env(pc_mapf))
        for c in get_all_conflicts(conflict_table)
            for id in [RobotID(c.agent1_id),RobotID(c.agent2_id)]
                for e in edges(bfs_tree(sched,id))
                    if needs_backtrack(solver,sched,get_vtx_id(sched,e.dst))
                        need_backtrack = true
                        break
                    end
                end
            end
        end
        if need_backtrack && activate_backtracking!(solver)
            reset_schedule_times!(get_schedule(search_env),get_schedule(get_env(pc_mapf)))
            reset_cache!(get_cache(search_env),get_schedule(search_env),)
            reset_route_plan!(node,get_route_plan(get_env(pc_mapf)))
            valid_flag = compute_route_plan!(solver, pc_mapf, node)
        end
    end
    return valid_flag
end
function CRCBS.low_level_search!(solver::CBSSolver, pc_mapf::AbstractPC_MAPF,node::ConstraintTreeNode,args...;kwargs...)
    low_level_search!(low_level(solver),pc_mapf,node,args...)
end

"""
    solve!(solver::ISPS,mapf::M) where {M<:PC_MAPF}

Solve a PC-MAPF problem with the ISPS algorithm.
"""

function CRCBS.solve!(solver::ISPS,pc_mapf::PC_MAPF)
    node = initialize_root_node(solver,pc_mapf)
    valid_flag = compute_route_plan!(solver,pc_mapf,node)
    return node.solution, get_cost(node.solution)
end

construct_cost_model(solver::CBSSolver,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::CBSSolver) = search_trait(low_level(solver))
primary_cost(solver::CBSSolver,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::CBSSolver) = primary_cost_type(low_level(solver))
CRCBS.solve!(solver::CBSSolver,pc_mapf::PC_MAPF) = CRCBS.cbs!(solver,pc_mapf)



export
    TaskGraphsMILPSolver

"""
    TaskGraphsMILPSolver

Wrapper for MILP solver for assignment problem.
"""
@with_kw struct TaskGraphsMILPSolver{M,C}
    milp::M = SparseAdjacencyMILP()
    logger::SolverLogger{C} = SolverLogger{Int}()
end
TaskGraphsMILPSolver(milp) = TaskGraphsMILPSolver(milp,SolverLogger{Int}())
get_assignment_matrix(solver::TaskGraphsMILPSolver) = get_assignment_matrix(solver.milp)

function formulate_milp(solver::TaskGraphsMILPSolver,args...;kwargs...)
    milp = formulate_milp(solver.milp,args...;
        kwargs...)
    set_optimizer_attributes(milp,default_optimizer_attributes()...)
    set_time_limit_sec(milp, max(0,min(1000,time_to_deadline(solver))))
    milp
end


export
    NBSSolver,
    assignment_solver,
    route_planner

"""
    NBSSolver{A,P}

A hierarchical PC-TAPF solver with an assignment level and a path-planning
level.
The solver works by alternating between assignment and path-planning until the
optimality gap between the lower bound (from task assignment) and the lower
bound (from path planning) disappears.
The input to the assignment problem is the full PC-TAPF problem specification.
The output of the assignment problem is a valid `OperatingSchedule`--that is,
an operating schedule wherein all assignments have been made in a legal way.
The input to the route planner is the PC-TAPF problem spec along with the
`OperatingSchedule` that comes from the assignment solution.
"""
@with_kw struct NBSSolver{A,P,C} <: AbstractPCTAPFSolver
    assignment_model::A     = TaskGraphsMILPSolver()
    path_planner    ::P     = CBSSolver(ISPS())
    logger          ::SolverLogger{C} = SolverLogger{primary_cost_type(path_planner)}()
    return_first_feasible::Bool        = false # if true, return the first feasible solution
end
NBSSolver(a,b) = NBSSolver(assignment_model=a,path_planner=b)
assignment_solver(solver::NBSSolver) = solver.assignment_model
route_planner(solver::NBSSolver) = solver.path_planner
construct_cost_model(solver::NBSSolver,args...;kwargs...) = construct_cost_model(route_planner(solver),args...;kwargs...)
search_trait(solver::NBSSolver) = search_trait(route_planner(solver))
primary_cost(solver::NBSSolver,cost) = primary_cost(route_planner(solver),cost)
primary_cost_type(solver::NBSSolver) = primary_cost_type(route_planner(solver))
function CRCBS.set_best_cost!(solver::NBSSolver,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(route_planner(solver),cost)
end
function CRCBS.hard_reset_solver!(solver::NBSSolver)
    hard_reset_solver!(get_logger(solver.logger))
    hard_reset_solver!(route_planner(solver))
end
function CRCBS.reset_solver!(solver::NBSSolver)
    reset_solver!(get_logger(solver))
    reset_solver!(route_planner(solver))
end
for op in [:set_deadline!,:set_runtime_limit!,:set_verbosity!]
    eval(quote
        CRCBS.$op(solver::NBSSolver,args...) = begin
            $op(get_logger(solver),args...)
            $op(assignment_solver(solver),args...)
            $op(route_planner(solver),args...)
        end
    end)
end
isps_queue_cost(solver::CBSSolver,args...) = isps_queue_cost(low_level(solver),args...)
isps_queue_cost(solver::NBSSolver,args...) = isps_queue_cost(route_planner(solver),args...)

"""
    solve!(solver, base_env::SearchEnv;kwargs...) where {A,P}

Use the planner defined by `solver` to solve the PC-TAPF problem encoded by
`base_env`. For solvers of type `NBSSolver`, the algorithm involves
repeatedly solving an assignment problem followed by a route-planning problem.
Within the generic `solve!` method it is possible to initialize an assignment
problem (the type is not constrained) and then modify it via
`update_assignment_problem!` prior to each new call to
`solve_assignment_problem!`. This is the approach taken for various MILP-based
assignment solvers. It is also possible to reconstruct the assignment problem
from scratch within each call to `solve_assignment_problem!`.

Arguments:
- solver <: AbstractPCTAPFSolver
- base_env::SearchEnv : a PC-TAPF problem

Outputs:
- best_env : a `SearchEnv` data structure that encodes a solution to the problem
- cost : the cost of the solution encoded by `best_env`
"""
function CRCBS.solve!(solver::NBSSolver, prob::E;kwargs...) where {E<:AbstractPC_TAPF}
    best_env = SearchEnv(get_env(prob),route_plan=initialize_route_plan(get_env(prob)))
    set_cost!(best_env,get_infeasible_cost(best_env))
    set_best_cost!(solver, primary_cost(solver, get_cost(best_env)))
    assignment_problem = formulate_assignment_problem(assignment_solver(solver),
        prob;kwargs...)
    try
        while optimality_gap(solver) > 0
            if solver.return_first_feasible && feasible_status(solver)
                @log_info(1,verbosity(solver),
                    "NBS: Returning feasible solution: optimality gap = $(optimality_gap(solver))")
                break
            end
            enforce_time_limit!(solver)
            update_assignment_problem!(assignment_solver(solver),
                assignment_problem, prob)
            schedule, l_bound = solve_assignment_problem!(
                assignment_solver(solver),
                assignment_problem,
                prob)
            set_lower_bound!(solver,l_bound)
            if optimality_gap(solver) > 0
                soft_reset_solver!(route_planner(solver)) # Will this break anything?
                env, cost = plan_route!(route_planner(solver),
                    schedule, prob;kwargs...)
                if cost < best_cost(solver)
                    best_env = env
                    set_best_cost!(solver,cost)
                end
            end
            increment_iteration_count!(solver)
            if check_iterations(solver)
                @log_info(1,verbosity(solver),
                    "NBS: Reached $(iteration_limit(solver))-iteration limit.")
                break
            end
        end
    catch e
        isa(e, SolverException) ? handle_solver_exception(solver,e) : rethrow(e)
    end
    return best_env, best_cost(solver)
end
function CRCBS.solve!(solver, env_graph, schedule::OperatingSchedule, problem_spec::ProblemSpec;
    kwargs...)
    base_env = construct_search_env(solver,schedule,problem_spec,env_graph;kwargs...)
    solve!(solver, base_env;kwargs...)
end
function CRCBS.solve!(solver, env_graph, project_spec::ProjectSpec, problem_spec::ProblemSpec,
        robot_ICs;kwargs...)
    schedule = construct_partial_project_schedule(
        project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    solve!(solver, env_graph, schedule, problem_spec;kwargs...)
end
CRCBS.solve!(solver::NBSSolver, env::SearchEnv;kwargs...) = solve!(solver,PC_TAPF(env);kwargs...)

export
    formulate_assignment_problem

"""
    formulate_assignment_problem(solver,prob;

Returns an assignment problem instance that can be updated (as opposed to being
reconstructed from scratch) on each call to `update_assignment_problem!` prior
to being resolved.
"""
function formulate_assignment_problem(solver,prob;
        kwargs...)
    formulate_milp(solver,get_env(prob);
        cost_model=get_problem_spec(get_env(prob)).cost_function,
        kwargs...)
end

function formulate_milp(milp_model::TaskGraphsMILP,env::SearchEnv;kwargs...)
    process_schedule!(get_schedule(env))
    t0,tF = get_node_start_and_end_times(env)
    formulate_milp(milp_model,get_schedule(env),get_problem_spec(env);
        t0_=t0,
        tF_=tF,
        kwargs...
        )
end
function formulate_milp(model::AbstractAssignmentMILP,env::SearchEnv;kwargs...)
    process_schedule!(get_schedule(env))
    formulate_milp(model,get_schedule(env),get_problem_spec(env);kwargs...)
end

export
    update_assignment_problem!

"""
    update_assignment_problem!(solver, assignment_problem)

A helper method for updating an instance of an assignment problem. In the case
    of MILP-based models, this method simply excludes all previous solutions by
    adding new constraints on the assignment/adjacency matrix.
"""
function update_assignment_problem!(solver, model::T, base_prob) where {T<:TaskGraphsMILP}
    exclude_solutions!(model) # exclude most recent solution in order to get next best solution
    # Trying this
    set_time_limit_sec(model, max(0,min(1000,time_to_deadline(solver))))
    model
end

export
    solve_assignment_problem!

"""
    solve_assignment_problem!(solver,model,prob)

Solve the "assignment problem"--i.e., the relaxation of the full PC-TAPF problem
wherein we ignore collisions--using the algorithm encoded by solver.
"""
function solve_assignment_problem!(solver::TaskGraphsMILPSolver, model, prob)
    l_bound = lower_bound(solver)
    set_time_limit_sec(model, max(0,min(1000,time_to_deadline(solver))))
    optimize!(model)
    if primal_status(model) != MOI.FEASIBLE_POINT
        throw(SolverException("Assignment problem is infeasible -- in `solve_assignment_problem!(solver::$(typeof(solver)))`. iterations = $(iterations(solver))"))
        # @warn "Assignment problem is infeasible -- in `solve_assignment_problem!(solver::$(typeof(solver)))`. iterations = $(iterations(solver))"
        # set_lower_bound!(solver, typemax(Int))
        # set_best_cost!(solver, typemax(Int))
        # return OperatingSchedule(), lower_bound(solver)
    end
    # @show value(objective_bound(model)), value(objective_function(model))
    set_lower_bound!(solver, Int(round(value(objective_bound(model)))) )
    set_best_cost!(solver, Int(round(value(objective_function(model)))) )
    if termination_status(model) == MOI.OPTIMAL
        @assert lower_bound(solver) <= best_cost(solver) "lower_bound($(solver_type(solver))) = $(value(objective_bound(model))) -> $(lower_bound(solver)) but should be equal to best_cost($(solver_type(solver))) = $(value(objective_function(model))) -> $(best_cost(solver))"
    else
        @warn "Assignment problem not solved to optimality"
    end
    sched = deepcopy(get_schedule(get_env(prob)))
    update_project_schedule!(solver, model, sched, get_problem_spec(get_env(prob)))
    return sched, lower_bound(solver)
end


function CRCBS.solve!(solver,pcta::PC_TA)
    prob = formulate_assignment_problem(solver,pcta)
    sched, l_bound = solve_assignment_problem!(solver,prob,pcta)
end

export
    plan_route!

"""
    plan_route!

Compute a route plan that corresponds to the OperatingSchedule.
Arguments:
- solver
- schedule
- search_env

Outputs:
- A `SearchEnv` the contains a valid solution
"""
function plan_route!(
        solver,
        schedule,
        prob;
        kwargs...)

    # soft_reset_solver!(solver) # Will this break anything?
    env = construct_search_env(solver, schedule, get_env(prob);kwargs...)
    pc_mapf = construct_routing_problem(prob,env)
    solution, cost = solve!(solver, pc_mapf)
    solution, primary_cost(solver,get_cost(solution))
end

CRCBS.solver_type(::NBSSolver)          = "NBSSolver"
CRCBS.solver_type(::CBSSolver)          = "CBSSolver"
CRCBS.solver_type(::PIBTPlanner)        = "PIBTPlanner"
CRCBS.solver_type(::ISPS)               = "ISPS"
CRCBS.solver_type(::AStarSC)            = "AStarSC"
# CRCBS.solver_type(::PrioritizedAStarSC) = "PrioritizedAStarSC"
CRCBS.solver_type(::AStar)              = "AStar"
CRCBS.solver_type(::TaskGraphsMILPSolver{M,C}) where {M,C} = "TaskGraphsMILPSolver{$M}"

# end

################################################################################
##############################  Multi Goal Solver  #############################
################################################################################

include("path_planners/multi_goal_astar_sc.jl")