export
    PlanningCache,
    isps_queue_cost,
    initialize_planning_cache,
    reset_cache!


@with_kw_noshow struct PlanningCache
    closed_set::Set{Int}                    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}                    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} = PriorityQueue{Int,Tuple{Int,Float64}}() # active nodes prioritized by slack
end
function sprint_cache(io::IO, cache::PlanningCache;label_pad=14,pad=5)
    lpad(str) = sprint_padded(str;pad=label_pad,leftaligned=true)
    rpad(str) = sprint_padded(str;pad=label_pad,leftaligned=false)
    spad(str;kwargs...) = sprint_padded_list(str;pad=pad,leftaligned=false,kwargs...)
    print(io,"PlanningCache:","\n")
    print(io,"\t",lpad("closed_set:  "),cache.closed_set,"\n")
    print(io,"\t",lpad("active_set:  "),cache.active_set,"\n")
    print(io,"\t",lpad("node_queue:  "),cache.node_queue,"\n")
end

function Base.show(io::IO, cache::PlanningCache)
    sprint_cache(io, cache)
end

function isps_queue_cost(sched::OperatingSchedule,v::Int)
    path_spec = get_path_spec(sched,v)
    return (Int(path_spec.plan_path), minimum(get_slack(sched,v)))
end

function initialize_planning_cache(sched::OperatingSchedule)
    cache = PlanningCache()
    for v in get_all_root_nodes(sched)
        push!(cache.active_set,v)
        enqueue!(cache.node_queue,v=>isps_queue_cost(sched,v)) # need to store slack
    end
    cache
end

"""
    `reset_cache!(cache,sched)`

    Resets the cache so that a solution can be repaired (otherwise calling
    low_level_search!() will return immediately because the cache says it's
    complete)
"""
function reset_cache!(cache::PlanningCache,sched::OperatingSchedule)
    process_schedule!(sched)
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    for v in vertices(get_graph(sched))
        if is_root_node(get_graph(sched),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(sched,v))
        end
    end
    cache
end

export
    EnvGraphDict,
    construct_env_graph_dict

const EnvGraphDict = Dict{Symbol,GridFactoryEnvironment}
function construct_env_graph_dict(sched::OperatingSchedule,env::GridFactoryEnvironment)
    dict = EnvGraphDict()
    dict[:Default] = env
    for v in vertices(sched)
        node = get_node_from_vtx(sched,v)
        k = graph_key(node)
        if !haskey(dict,k)
            dict[k] = GridFactoryEnvironment(
                env,
                graph=deepcopy(env.graph),
                dist_function=deepcopy(get_dist_matrix(env))
            )
        end
    end
    return dict
end

function initialize_route_plan end

include("pccbs.jl")

export
    construct_cost_model,
    construct_heuristic_model

function construct_cost_model end
function construct_heuristic_model end

export
    EnvironmentLayer,
    SearchEnv,
    construct_search_env,
    update_env!

@with_kw_noshow struct EnvironmentLayer
    graph::GridFactoryEnvironment   = GridFactoryEnvironment()
    problem_spec::ProblemSpec       = ProblemSpec()
end
GraphUtils.get_graph(layer::EnvironmentLayer)            = layer.graph
get_problem_spec(layer::EnvironmentLayer)           = layer.problem_spec
CRCBS.get_cost_model(layer::EnvironmentLayer)       = layer.cost_model
CRCBS.get_heuristic_model(layer::EnvironmentLayer)  = layer.heuristic_model

function construct_environment_layer(env::GridFactoryEnvironment,problem_spec::ProblemSpec)
    env_graph = GridFactoryEnvironment(env, graph=deepcopy(env.graph),
        dist_function=deepcopy(get_dist_matrix(env))
        )
    prob_spec = ProblemSpec(problem_spec, D=env_graph)
    EnvironmentLayer(env_graph,prob_spec)
end

function populate_environment_layer_dict!(layers,
        sched::OperatingSchedule,
        env::GridFactoryEnvironment,
        prob_spec::ProblemSpec,
    )
    gkeys = unique(map(graph_key, map(n->n.node, sched.nodes)))
    for k in gkeys
        layers[k] = construct_environment_layer(env,prob_spec)
    end
    return layers
end
function construct_environment_layer_dict(args...)
    layers = Dict{Symbol,EnvironmentLayer}()
    populate_environment_layer_dict!(layers,args...)
end

"""
    SearchEnv{C,H,S} <: GraphEnv{State,Action,C}

Contains all of the information needed to fully define `PC_TAPF` and related
problems.
"""
@with_kw_noshow struct SearchEnv{C,H,S} <: GraphEnv{State,Action,C}
    schedule::OperatingSchedule     = OperatingSchedule()
    cache::PlanningCache            = PlanningCache()
    # Each agent type needs its own set of these #
    env_layers::Dict{Symbol,EnvironmentLayer} = Dict{Symbol,EnvironmentLayer}()
    # ###################################### #
    cost_model::C                   = C()
    heuristic_model::H              = H()
    num_agents::Int                 = length(get_robot_ICs(schedule))
    route_plan::S                   = initialize_route_plan(schedule,cost_model)
end

export
    get_schedule,
    get_cache,
    get_problem_spec,
    get_route_plan

GraphUtils.get_graph(env::SearchEnv,k=graph_key())     = get_graph(env.env_layers[k])
get_problem_spec(env::SearchEnv,k=graph_key())         = get_problem_spec(env.env_layers[k])
CRCBS.get_cost_model(env::SearchEnv)        = env.cost_model
CRCBS.get_heuristic_model(env::SearchEnv)   = env.heuristic_model
get_schedule(env::SearchEnv) = env.schedule
get_schedule(sched::OperatingSchedule) = sched
get_cache(env::SearchEnv) = env.cache
get_route_plan(env::SearchEnv) = env.route_plan

for op in path_spec_accessor_interface
    @eval $op(env::SearchEnv,v) = $op(get_schedule(env),v)
end
for op in path_spec_mutator_interface
    @eval $op(env::SearchEnv,v,val) = $op(get_schedule(env),v,val)
end

function CRCBS.get_start(env::SearchEnv,v::Int)
    node = get_node(get_schedule(env),v)
    start_vtx = get_id(get_default_initial_location_id(node))
    # start_vtx   = get_path_spec(get_schedule(env),v).start_vtx
    start_time  = get_t0(env,v)
    state_type(env)(start_vtx,start_time)
end
CRCBS.num_agents(env::SearchEnv) = env.num_agents
for op in [
    :cost_type,:state_type,:action_type,:path_type,:get_cost,:get_paths,
    :get_path_costs,:set_cost!,:set_solution_path!,:set_path_cost!,
    :convert_to_vertex_lists,:detect_conflicts,:detect_conflicts!
    ]
    @eval CRCBS.$op(env::SearchEnv,args...) = $op(get_route_plan(env),args...)
end
function sprint_search_env(io::IO,env::SearchEnv)
    # print(io,"schedule: ",get_schedule(env),"\n")
    print(io,"SearchEnv: \n")
    print(io,"cache: ",sprint(sprint_cache,get_cache(env)))
    print(io,"active task nodes:","\n")
    for v in get_cache(env).active_set
        print(io,"\t","v = ",
            sprint_padded(v)," => ",
            string(get_node_from_vtx(get_schedule(env),v)),"\n")
    end
    print(io,"route_plan: ",get_route_plan(env),"\n")
end
function Base.show(io::IO,env::SearchEnv)
    sprint_search_env(io,env)
end

export
    get_env_snapshot,
    trim_route_plan

"""
    get_env_snapshot(route_plan::S,t)
"""
function get_env_snapshot(env::SearchEnv,t)
    sched = get_schedule(env)
    route_plan = get_route_plan(env)
    Dict(id=>BOT_AT(id,
        LocationID(get_sp(get_path_node(get_paths(route_plan)[get_id(id)],t)).vtx)
        ) for (id,r) in get_robot_ICs(sched))
end

export
    EnvState,
    get_env_state

"""
    EnvState

Reflects the state of the SearchEnv environment at a given time step.
"""
@with_kw_noshow struct EnvState
    robot_positions::Dict{BotID,BOT_AT} = Dict{BotID,BOT_AT}()
    object_positions::Dict{ObjectID,OBJECT_AT} = Dict{ObjectID,OBJECT_AT}()
    objects_active::Dict{ObjectID,Bool} = Dict{ObjectID,Bool}()
end

export
    robot_positions,
    object_positions,
    objects_active,
    object_position,
    object_active,
    robot_position

robot_positions(s::EnvState) = s.robot_positions
object_positions(s::EnvState) = s.object_positions
objects_active(s::EnvState) = s.objects_active
object_position(s::EnvState,k) = object_positions(s)[k]
robot_position(s::EnvState,k) = robot_positions(s)[k]
object_active(s::EnvState,k) = objects_active(s)[k]

function Base.show(io::IO,s::EnvState)
    print(io,"EnvState")
    for (n,dict) in [
        ("robot_positions",robot_positions(s)),
        ("object_positions",object_positions(s)),
        ("objects_active",objects_active(s)),
        ]
        print(io,"\n\t",n," : ")
        for k in sort(collect(keys(dict)))
            val = dict[k]
            print(io,string(val),", ")
        end
    end
end
function get_env_state(env,t)
    sched = get_schedule(env)
    # cache = get_cache(env)
    robot_positions = get_env_snapshot(env,t)
    object_positions = Dict{ObjectID,OBJECT_AT}()
    objects_active = Dict{ObjectID,Bool}()

    #########
    # This block was added by Kyle in commits on the develop branch with commit 6b4ebf2.
    # The commit message was "Final changes for thesis." However, it does not appear to work
    # in finding the correct objects as it only considers an object if it matches the BOT_CARRY
    # template. This resulted in failing 6 tests in test_path_planning.jl. Resorting back to
    # the original code for now.
    #########
    # for n in get_nodes(sched)
    #     if matches_template(BOT_CARRY,n)
    #         o = get_object_id(n)
    #         o_node = get_node(sched,o)
    #         v_deposit = first(outneighbors(sched,n))
    #         v_collect = first(inneighbors(sched,n))
    #         if t <= get_t0(n)
    #             object_positions[o] = o_node.node
    #             objects_active[o] = t >= get_t0(o_node)
    #         else
    #             object_positions[o] = o_node.node
    #             node = get_node_from_vtx(sched,v_deposit)
    #             if t >= get_t0(env,v_deposit)
    #                 object_positions[o] = OBJECT_AT(o,
    #                     map(n->get_location_id(n),sub_nodes(node))
    #                     )
    #             elseif t >= get_t0(env,v_collect)
    #                 r_ids = get_valid_robot_ids(node)
    #                 object_positions[o] = OBJECT_AT(o,
    #                     map(r->get_location_id(robot_positions[r]),r_ids)
    #                     )
    #             end
    #             objects_active[o] = (t <= get_tF(env,v_deposit))
    #         end
    #     end
    # end
    for (o,o_node) in get_object_ICs(sched)
        v = get_vtx(sched,o)
        if t <= get_t0(env,v)
            object_positions[o] = o_node
            objects_active[o] = t >= get_t0(env,v)
        else
            object_positions[o] = o_node
            vtxs = capture_connected_nodes(
                sched,v,v->check_object_id(get_node_from_vtx(sched,v),o)
                )
            node_ids = map(v->get_vtx_id(sched,v),collect(vtxs))
            # @show node_ids
            v_deposit = filter(v->isa(get_node_from_vtx(sched,v),
            Union{BOT_DEPOSIT,TEAM_DEPOSIT}),
                collect(vtxs))[1]
            v_collect = filter(v->isa(get_node_from_vtx(sched,v),
            Union{BOT_COLLECT,TEAM_COLLECT}),
                collect(vtxs))[1]

            node = get_node_from_vtx(sched,v_deposit)
            if t >= get_t0(env,v_deposit)
                object_positions[o] = OBJECT_AT(o,
                    map(n->get_location_id(n),sub_nodes(node))
                    )
            elseif t >= get_t0(env,v_collect)
                r_ids = get_valid_robot_ids(node)
                object_positions[o] = OBJECT_AT(o,
                    map(r->get_location_id(robot_positions[r]),r_ids)
                    )
            # else
            #     r_ids = get_valid_robot_ids(node)
            #     @show object_positions[o] = OBJECT_AT(o,
            #         map(r->get_location_id(robot_positions[r]),r_ids)
            #         )
            end
            objects_active[o] = (t <= get_tF(env,v_deposit))
        end
    end
    EnvState(
        robot_positions = robot_positions,
        object_positions = object_positions,
        objects_active = objects_active
    )
end

export get_node_start_and_end_times
"""
    get_start_and_end_maps(sched,cache,default=0)

Return dictionaries mapping each node id in schedule to its start and end time
"""
function get_node_start_and_end_times(sched::OperatingSchedule)
    t0 = Dict{AbstractID,Int}(get_vtx_id(sched,v)=>get_t0(sched, v) for v in vertices(sched))
    tF = Dict{AbstractID,Int}(get_vtx_id(sched,v)=>get_tF(sched, v) for v in vertices(sched))
    return t0,tF
end
get_node_start_and_end_times(env::SearchEnv) = get_node_start_and_end_times(get_schedule(env))


export
    AbstractPC_MAPF,
    PC_MAPF,
    C_PC_MAPF,
    AbstractPC_TAPF,
    PC_TAPF,
    PC_TA,
    C_PC_TAPF

export
    construct_routing_problem

"""
    AbstractPC_MAPF

An abstract type of which all Precedence-Constrained Multi-Agent Path-Finding
problems are concrete subtypes.
"""
abstract type AbstractPC_MAPF <: AbstractMAPF end

"""
    AbstractPC_TAPF <: AbstractPC_MAPF

An abstract type of which all Precedence-Constrained Multi-Agent Task Assignment
and Path-Finding problems are concrete subtypes.
"""
abstract type AbstractPC_TAPF <: AbstractPC_MAPF end

"""
    PC_TAPF

Precedence-Constrained Multi-Agent Task Assignment and Path-Finding problem.
"""
struct PC_TAPF{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end

"""
    PC_TA

Precedence-Constrained Multi-Agent Task Assignment problem (no route planning).
"""
struct PC_TA{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end

"""
    PC_MAPF

A precedence-constrained multi-agent path-finding problem (no task assignment).
"""
struct PC_MAPF{E<:SearchEnv} <: AbstractPC_MAPF
    env::E
end
construct_routing_problem(prob::PC_TAPF,env) = PC_MAPF(env)

"""
    C_PC_MAPF

A collaborative precedence-constrained multi-agent path-finding problem. All
agents have assigned tasks, there are precedence constraints between tasks, and
some tasks must be done in teams.
"""
struct C_PC_MAPF{E<:SearchEnv} <: AbstractPC_MAPF
    env::E
end

"""
    C_PC_TAPF

Defines an instance of a Collaborative Precedence-Constrained Multi-Agent Task
    Assignment and Path-Finding problem, where agents must sometimes transport
    objects in teams.
"""
struct C_PC_TAPF{E<:SearchEnv} <: AbstractPC_TAPF
    env::E
end
C_PC_TAPF(p::PC_TAPF) = C_PC_TAPF(p.env)
construct_routing_problem(prob::C_PC_TAPF,env) = C_PC_MAPF(env)

for T in [:PC_TAPF,:PC_MAPF,:PC_TA]
    @eval $T(prob::$T,env::SearchEnv) = $T(env)
end


initialize_planning_cache(env::SearchEnv) = initialize_planning_cache(get_schedule(env))

"""
    get_next_vtx_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_vtx_matching_agent_id(env::SearchEnv,agent_id)
    @assert isa(agent_id,BotID)
    for v in get_cache(env).active_set
        n = get_node(get_schedule(env),v)
        has_robot_id(n) ? nothing : continue
        if agent_id == get_robot_id(n)
            return v
        end
    end
    return -1
end

"""
    get_next_node_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_node_matching_agent_id(env::SearchEnv,agent_id)
    @assert isa(agent_id,BotID)
    v = get_next_vtx_matching_agent_id(env,agent_id)
    if has_vertex(get_schedule(env),v)
        return get_vtx_id(get_schedule(env),v)
    end
    return agent_id
end

export
    update_planning_cache!

"""
    update_planning_cache!(solver,env,v,path)
"""
update_planning_cache!(solver,env::SearchEnv,v::Int,path::Path) = update_planning_cache!(solver,env,v,get_t(get_final_state(path)))
function update_planning_cache!(solver,env::SearchEnv,v::Int,t::Real=-1)
    update_planning_cache!(solver,get_schedule(env),get_cache(env),v,t)
end
function update_planning_cache!(solver,sched::OperatingSchedule,cache::PlanningCache,v::Int,t=-1)
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    Δt = t - get_tF(sched,v)
    if Δt > 0
        set_tF!(sched,v,t)
        process_schedule!(sched)
    end
    # update closed_set
    activated_vtxs = Int[]
    push!(closed_set,v)
    # update active_set
    setdiff!(active_set, v)
    for v2 in outneighbors(sched,v)
        active = true
        for v1 in inneighbors(sched,v2)
            if !(v1 in closed_set)
                active = false
                break
            end
        end
        if active
            push!(activated_vtxs,v2)
            push!(active_set, v2)               # add to active set
        end
    end
    # update priority queue
    for v2 in active_set
        node_queue[v2] = isps_queue_cost(sched,v2)
    end
    @log_info(2,verbosity(solver),"moved ",v," to closed set, moved ",activated_vtxs," to active set")
    @log_info(3,verbosity(solver),string("get_tF(sched,v) = ",get_tF(sched,v)))
    return cache
end
"""
    update_planning_cache!(solver,env)

Update cache continually. After a call to this function, the start and end times
of all schedule nodes will be updated to reflect the progress of active schedule
nodes (i.e., if a robot had not yet completed a GO task, the predicted final
time for that task will be updated based on the robot's current state and
distance to the goal).
All active nodes that don't require planning will be automatically marked as
complete.
"""
function update_planning_cache!(solver,env)
    cache = get_cache(env)
    sched = get_schedule(env)
    # Skip over nodes that are already planned or just don't need planning
    while true
        done = true
        for v in collect(cache.active_set)
            if get_path_spec(sched,v).plan_path == false
                update_planning_cache!(solver,env,v)
                done = false
            end
        end
        if done
            break
        end
    end
    for v in collect(cache.active_set)
        n = get_node(sched,v)
        agent_id = get_id(get_default_robot_id(n))
        # path_spec = get_path_spec(sched,v)
        if 1 <= agent_id <= num_agents(env)
            path = get_paths(env)[agent_id]
            s = get_final_state(path)
            t = get_t(s)
            goal_vtx = get_id(get_destination_location_id(n))
            goal_state = state_type(env)(goal_vtx,-1)
            d = get_distance(env,s,goal_state)
            set_tF!(env,v,max(get_tF(env,v),t+d))
        end
    end
    process_schedule!(sched)
    cache
end

for op in [:SumOfMakeSpans,:MakeSpan]
    @eval $op(sched::OperatingSchedule,cache::PlanningCache) = $op(
        get_tF(sched),
        sched.terminal_vtxs,
        map(k->sched.weights[k], sched.terminal_vtxs),
        map(v->get_tF(sched,v),sched.terminal_vtxs),
    )
end

function initialize_route_plan(sched::OperatingSchedule,cost_model)
    starts = State[]
    robot_ics = get_robot_ICs(sched)
    for k in sort(collect(keys(robot_ics)))
        push!(starts, State(vtx = get_id(get_location_id(robot_ics[k])), t = 0))
    end
    paths = map(s->Path{State,Action,cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end
function initialize_route_plan(env::SearchEnv)
    cost_model=get_cost_model(env)
    paths=deepcopy(get_paths(get_route_plan(env)))
    costs = map(p->get_cost(p), paths)
    LowLevelSolution(
        paths=paths,
        cost_model=cost_model,
        costs=costs,
        cost=aggregate_costs(cost_model, costs)
        )
end
function initialize_route_plan(env::SearchEnv,cost_model)
    paths = [Path(
        path_nodes=deepcopy(p.path_nodes),
        s0=p.s0,
        cost = compute_path_cost(cost_model,get_graph(env),p,i)
    ) for (i,p) in enumerate(get_paths(get_route_plan(env)))]
    costs=map(p->get_cost(p),paths)
    LowLevelSolution(
        paths=paths,
        cost_model=cost_model,
        costs=costs,
        cost=aggregate_costs(cost_model, costs)
        )
end
function construct_search_env(
        solver,
        sched::OperatingSchedule,
        problem_spec::ProblemSpec,
        env_graph,
        cache::PlanningCache=initialize_planning_cache(sched),
        primary_objective=problem_spec.cost_function,
        ;
        extra_T=2000,
        kwargs...
    )
    cost_model, heuristic_model = construct_cost_model(
        solver,
        sched,
        cache,
        problem_spec,
        env_graph,
        primary_objective;
        extra_T=extra_T,
        )
    layers = construct_environment_layer_dict(sched,env_graph,problem_spec)
    search_env = SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=layers,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
        )
    return search_env
end
"""
    construct_search_env(solver,schedule,env,...)

Constructs a new search env by combining the new `schedule` with the pre-
existing `get_route_plan(env)`. This involves constructing a new cost function that
reflects the new schedule structure.
TODO: Carry over information about `get_cache(search_env)`
"""
function construct_search_env(
        solver,
        sched::OperatingSchedule,
        env::SearchEnv,
        cache::PlanningCache=initialize_planning_cache(sched),
        env_graph=get_graph(env),
        problem_spec = get_problem_spec(env),
        primary_objective = problem_spec.cost_function,
        args...
        ;
        extra_T=2000,
        kwargs...
    )
    cost_model, heuristic_model = construct_cost_model(
        solver,
        sched,
        cache,
        problem_spec,
        env_graph,
        primary_objective;
        extra_T=extra_T,
        )
    layers = construct_environment_layer_dict(sched,env_graph,problem_spec)
    route_plan = initialize_route_plan(env,cost_model)
    N = length(get_paths(route_plan))
    search_env = SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=layers,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
        num_agents=N,
        route_plan=route_plan)
end

"""
    function construct_search_env(solver, env::SearchEnv, ... )

Construct a new SearchEnv, with cost_model and heuristic_model defined by the
solver type.
"""
function construct_search_env(solver,
    env::SearchEnv,
    sched=get_schedule(env),
    cache=get_cache(env),
    env_graph=get_graph(env),
    prob_spec=get_problem_spec(env),
    primary_objective = problem_spec.cost_function,
    ;
    kwargs...
    )
    cost_model, heuristic_model = construct_cost_model(
        solver, sched, cache, prob_spec, env_graph, primary_objective;kwargs...
    )
    SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=env.env_layers,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
    )

end

update_cost_model!(model::C,env::S) where {C,S<:SearchEnv} = nothing
function update_cost_model!(model::C,env::S) where {C<:MultiDeadlineCost,S<:SearchEnv}
    model.tF .= get_tF(get_schedule(env))
end
function update_cost_model!(model::C,env::S) where {C<:CompositeCostModel,S<:SearchEnv}
    for m in model.cost_models
        update_cost_model!(m,env)
    end
end
update_cost_model!(env::S) where {S<:SearchEnv} = update_cost_model!(get_cost_model(env),env)

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver,env::SearchEnv,v::Int,path::P,
        agent_id = get_id(get_default_robot_id(get_node(get_schedule(env),v)))
        ) where {P<:Path}
    route_plan = get_route_plan(env)
    # UPDATE CACHE
    update_planning_cache!(solver,env,v,get_t(get_final_state(path)))
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    # n = get_node(get_schedule(env),v)
    # agent_id = has_robot_id(n) ? get_id(get_robot_id(n)) : -1
    # agent_id = get_id(get_default_robot_id(n))
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
        partially_set_path!(get_cost_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
    end

    env
end

################################################################################
############################### Low Level Search ###############################
################################################################################


export evaluate_path_gap

"""
    evaluate_path_gap(search_env::SearchEnv,path,v)

Returns the gap between a path's length and it's expected length (based on times
stored in `get_cache(env).t0`)
"""
function evaluate_path_gap(search_env::SearchEnv,path,v)
    t0 = get_t0(search_env,v)
    gap = t0 - get_end_index(path)
    return gap
end

function replan_path!(solver, pc_mapf::AbstractPC_MAPF, env::SearchEnv, constraint_node::ConstraintTreeNode, vtx::VtxID,t0)
    v = get_id(vtx)
    sched = get_schedule(env)
    n_id = get_vtx_id(sched,v)
    schedule_node = get_node_from_id(sched,n_id)
    ### trim path
    cbs_env = build_env(solver,pc_mapf,env,constraint_node,VtxID(v))
    path = get_base_path(solver,env,cbs_env)
    trim_path!(cbs_env,path,get_t0(env,v))
    ### plan path with new goal time
    status = plan_path!(low_level(solver),pc_mapf,env,constraint_node,schedule_node,v)
    return status
end

export tighten_gaps!

"""
    tighten_gaps!(solver, pc_mapf, env::SearchEnv, constraint_node::ConstraintTreeNode)

If any path ends before it should (based on times stored in `get_cache(env)`),
recomputes the path segment for the final node in that line.
"""
function tighten_gaps!(solver, pc_mapf::AbstractPC_MAPF, env::SearchEnv, constraint_node::ConstraintTreeNode)
    solver.tighten_paths ? nothing : return env
    sched = get_schedule(env)
    active_nodes = robot_tip_map(sched,get_cache(env).active_set)
    for (robot_id, n_id) in active_nodes
        path = get_paths(env)[get_id(robot_id)]
        v = get_vtx(sched,n_id)
        gap = evaluate_path_gap(env,path,v)
        if gap > 0
            t0 = get_t0(env,v)
            @log_info(2, verbosity(solver), "tighten_gaps!: base path for ",
                string(get_node_from_vtx(sched,v)),
                ", v = ",v," ends at t=",get_end_index(path),
                " but should end at t=",t0," (gap = ", gap,").")
            vtxs = backtrack_node(sched,v)
            for vp in vtxs
                if get_tF(env,vp) < t0
                    @log_info(2,verbosity(solver)," Re-launching planner on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0," ...")
                    if replan_path!(solver, pc_mapf, env, constraint_node, VtxID(vp),t0)
                        @log_info(2,verbosity(solver),"tightening succeeded on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0)
                    else
                        @log_info(2,verbosity(solver),"tightening failed on ",string(get_node_from_vtx(sched,vp))," (v = ",v,")"," with extended horizon ",t0)
                        return false
                    end
                end
            end
        end
    end
    return true
end

function get_base_path(solver,search_env::SearchEnv,env::PCCBSEnv)
    base_path = get_paths(search_env)[get_agent_id(env)]
    return base_path
end
function CRCBS.build_env(
        solver,
        pc_mapf::AbstractPC_MAPF,
        env::SearchEnv,
        constraint_node::ConstraintTreeNode,
        node::T,
        v::Int,
        path_spec=get_path_spec(get_schedule(env), v),
        ;
        heuristic = get_heuristic_model(env),
        cost_model = get_cost_model(env)
    ) where {T}
    agent_id= get_id(get_default_robot_id(node))
    # goal_vtx = path_spec.final_vtx
    goal_vtx = has_robot_id(node) ? get_id(get_destination_location_id(node)) : -1
    goal_time = get_tF(env,v) # time after which goal can be satisfied
    deadline = get_tF(env,v) .+ get_slack(env,v)         # deadline for DeadlineCost
    # Adjust deadlines if necessary:
    if path_spec.tight == true
        goal_time += minimum(get_local_slack(env,v))
    end
    for v_next in outneighbors(get_graph(get_schedule(env)),v)
        if get_path_spec(get_schedule(env), v_next).static == true
            duration_next = get_path_spec(get_schedule(env),v_next).min_duration
            for c in sorted_state_constraints(env,get_constraints(constraint_node, agent_id)) #.sorted_state_constraints
                if get_sp(get_path_node(c)).vtx == goal_vtx
                    if 0 < get_time_of(c) - goal_time < duration_next
                        @log_info(1,verbosity(solver),"extending goal_time for node ",v,
                            " from ",goal_time," to ",get_time_of(c),
                            " to avoid constraints")
                        goal_time = max(goal_time, get_time_of(c))
                    end
                end
            end
        end
    end
    if (path_spec.free == true) && is_terminal_node(get_graph(get_schedule(env)),v)
        goal_time = makespan(get_schedule(env))
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        @log_info(3,verbosity(solver),string("BUILD ENV: ",string(node),
            " - setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    @assert goal_time != Inf "goal time set to $goal_time for node $(string(node))"
    cbs_env = PCCBSEnv(
        search_env = env,
        schedule_node = node,
        node_id     = get_vtx_id(get_schedule(env),v),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        constraints = get_constraints(constraint_node, agent_id), # agent_id represents the whole path
        goal        = state_type(env)(goal_vtx,goal_time),
        cost_model  = cost_model,
        heuristic   = heuristic,
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    @log_info(3,verbosity(solver),"build_env: ",string(node),
        ", goal_time=",goal_time,", deadline=",deadline)
    return cbs_env
end
function CRCBS.build_env(
    solver,
    pc_mapf::AbstractPC_MAPF,
    env::SearchEnv,
    constraint_node::ConstraintTreeNode,
    v::VtxID,
    node=get_node_from_vtx(get_schedule(env),get_id(v)),
    args...
    ;
    kwargs...
    )
    id = get_id(v)
    env = build_env(solver,pc_mapf,env,constraint_node,node,id,args...;kwargs...)
end
function CRCBS.build_env(
        solver,
        pc_mapf::AbstractPC_MAPF,
        env::SearchEnv,
        constraint_node::ConstraintTreeNode,
        agent_id::AgentID
        )
    n_id = get_next_node_matching_agent_id(env,RobotID(get_id(agent_id)))
    build_env(solver,pc_mapf,env,constraint_node,VtxID(get_vtx(get_schedule(env),n_id)))
end
CRCBS.build_env(env::SearchEnv) = PCCBSEnv(search_env=env)

function get_base_path(solver,search_env::SearchEnv,meta_env::MetaAgentCBS.AbstractMetaEnv)
    base_paths = map(env->get_base_path(solver,search_env,env),MetaAgentCBS.get_envs(meta_env))
    cost = aggregate_costs(get_cost_model(meta_env), map(get_cost,base_paths))
    MetaAgentCBS.construct_meta_path(base_paths,cost)
end

"""
    For COLLABORATIVE transport problems
"""
function CRCBS.build_env(
    solver,
    pc_mapf::C_PC_MAPF,
    env::E,
    constraint_node::N,
    node::T,
    v::Int,
    ;
    kwargs...) where {E<:SearchEnv,N<:ConstraintTreeNode,T}
    envs = []
    agent_idxs = Int[]
    for (i, sub_node) in enumerate(sub_nodes(node))
        ph = PerfectHeuristic(get_team_config_dist_function(get_graph(env),team_configuration(node),i))
        heuristic = construct_heuristic_model(solver,get_graph(env),ph)
        cbs_env = build_env(solver,PC_MAPF(get_env(pc_mapf)),env,constraint_node,VtxID(v),sub_node,generate_path_spec(get_schedule(env),get_problem_spec(env),sub_node);
            heuristic=heuristic,
            kwargs...
        )
        push!(envs, cbs_env)
        push!(agent_idxs, get_id(get_robot_id(sub_node)))
    end
    meta_env = MetaAgentCBS.construct_team_env(
        [envs...],
        agent_idxs,
        get_cost_model(env)
        )
    return meta_env
end

export
    reset_route_plan!

"""
    Helper to reset the solution in a constraint node between re-runs of ISPS
"""
function reset_route_plan!(node::N,base_route_plan) where {N<:ConstraintTreeNode}
    for (agent_id,path) in enumerate(get_paths(base_route_plan))
        set_solution_path!(node.solution, deepcopy(path), agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
    end
    set_cost!(node, get_cost(base_route_plan))
    node
end

################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

CRCBS.build_env(solver, pc_mapf::AbstractPC_MAPF, args...) = build_env(solver,pc_mapf,get_env(pc_mapf),args...)
CRCBS.build_env(prob::Union{PC_TAPF,PC_MAPF}) = build_env(get_env(prob))
CRCBS.get_initial_solution(pc_mapf::PC_MAPF) = get_env(pc_mapf)
function CRCBS.initialize_root_node(env::SearchEnv)
    solution = SearchEnv(
        env,
        cache=initialize_planning_cache(env),
        route_plan=initialize_route_plan(env)
        )
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            i=>discrete_constraint_table(env,i) for i in 1:num_agents(env)
            ),
        id = 1)
end
function CRCBS.discrete_constraint_table(env::SearchEnv,agent_id=-1,tf=2*makespan(get_schedule(env))+100*num_agents(env))
    discrete_constraint_table(num_states(env),num_actions(env),agent_id,tf)
end
# CRCBS.initialize_root_node(solver,pc_mapf::AbstractPC_MAPF) = initialize_root_node(get_env(pc_mapf))
CRCBS.initialize_root_node(solver,pc_mapf::AbstractPC_MAPF) = initialize_root_node(copy(get_env(pc_mapf)))
function Base.copy(env::SearchEnv)
    SearchEnv(
        env,
        schedule=deepcopy(get_schedule(env)),
        cache=deepcopy(get_cache(env)),
        route_plan=deepcopy(get_route_plan(env))
        )
end
function CRCBS.default_solution(env::SearchEnv)
    solution = deepcopy(env)
    set_cost!(get_route_plan(solution),get_infeasible_cost(get_route_plan(solution)))
    solution, get_cost(solution)
end
CRCBS.default_solution(pc_mapf::M) where {M<:AbstractPC_MAPF} = default_solution(get_env(pc_mapf))
function CRCBS.cbs_update_conflict_table!(solver,mapf::AbstractPC_MAPF,node,constraint)
    search_env = node.solution
    idxs = collect(1:num_agents(search_env))
    # t0 = max(minimum(get_cache(search_env).t0), 1) # This is particularly relevant for replanning, where we don't care to look for conflicts way back in the past.
    t0 = max(minimum(get_t0(get_schedule(search_env))),1) # This is particularly relevant for replanning, where we don't care to look for conflicts way back in the past.
    detect_conflicts!(node.conflict_table,get_route_plan(search_env),idxs,Int(floor(t0)))
end
CRCBS.detect_conflicts!(table,env::SearchEnv,args...) = detect_conflicts!(table,get_route_plan(env),args...)

# CRCBS.serialize(pc_mapf::PC_MAPF,args...) = serialize(get_env(pc_mapf),args...)
# CRCBS.deserialize(pc_mapf::PC_MAPF,args...) = serialize(get_env(pc_mapf),args...)
CRCBS.get_env(pc_mapf::AbstractPC_MAPF)             = pc_mapf.env
for op in [
    :cost_type,:state_type,:action_type,:path_type,:num_states,:num_actions,
    :num_agents,:serialize,:deserialize,
    :discrete_constraint_table,
    ]
    @eval CRCBS.$op(prob::AbstractPC_MAPF,args...) = $op(get_env(prob),args...)
end
CRCBS.base_env_type(pc_mapf::AbstractPC_MAPF)               = PCCBSEnv
CRCBS.base_env_type(pc_mapf::Union{C_PC_MAPF,C_PC_TAPF})    = MetaAgentCBS.TeamMetaEnv

function validate(env::SearchEnv;kwargs...)
    validate(get_schedule(env),convert_to_vertex_lists(get_route_plan(env));kwargs...)
end
