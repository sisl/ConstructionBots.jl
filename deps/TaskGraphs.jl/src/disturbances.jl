export
    AbstractDisturbance,
    DisturbanceSequence

abstract type AbstractDisturbance end
const DisturbanceSequence = Vector{Pair{Int,AbstractDisturbance}}


export
    StochasticProblem,
    stochastic_problem

"""
    StochasticProblem{P<:AbstractPC_TAPF}

Defines a stochastic version of PC_TAPF, wherein different disturbances can
cause unexpected problems in the factory.
"""
struct StochasticProblem{P<:AbstractPC_TAPF}
    prob::P
    disturbances::DisturbanceSequence
end
CRCBS.get_env(spc_tapf::StochasticProblem) = get_env(spc_tapf.prob)

function stochastic_problem(ptype::Type{P},solver,prob,clean_up_bot_ICS,disturbances) where {P<:AbstractPC_MAPF}
    env = get_env(deepcopy(prob))
    env_graph = get_graph(env)
    sched = get_schedule(env)
    cache = get_cache(env)
    for pred in clean_up_bot_ICS
        add_new_robot_to_schedule!(sched,pred,get_problem_spec(env))
    end
    robot_ICs = get_robot_ICs(sched)
    prob_spec = get_problem_spec(env)
    # populate_environment_dict_layers!(env.env_layers,sched,env_graph,prob_spec)
    new_prob = P(prob,construct_search_env(solver,sched,prob_spec,env_graph,cache))
    StochasticProblem(new_prob,disturbances)
end

export OilSpill
"""
    OilSpill

An obstruction that affects vertices `vtxs` and edges `edges`.
"""
struct OilSpill <: AbstractDisturbance
    vtxs::Set{Int}
    edges::Set{Edge}
end
function OilSpill(G::AbstractGraph,vtxs::Vector{Int})
    OilSpill(Set(vtxs),edge_cover(G,vtxs))
end

export Intruder
"""
    Intruder

An intruder that begins at location `start_vtx` and follows policy `policy`
"""
struct Intruder{P} <: AbstractDisturbance
    policy::P
    start_vtx::Int # start vtx
end

export DeadRobot
"""
    DeadRobot

Robot `id` is frozen.

Effect:
- Freeze robot
- Add "no-go" constraint to CBS/PIBT (how to do consistently? Perhaps place in
    SearchEnv and add directly to PCCBSEnv) OR temporarily remove vertex from
    graph
- Set robot state to NULL state? How to avoid having CBS complain about
    conflicts? Maybe set State to NULL State and place DeadRobotObject at the
    collection site?
- Dispatch CleanUpBot to collect frozen robot
- When CleanUpBot returns to "garage", regenerate frozen Robot's ROBOT_AT node
    and valid state.
"""
struct DeadRobot <: AbstractDisturbance
    id::Int
end

export DelayedRobot
"""
    DelayedRobot

Robot `id` is delayed by `dt` timesteps.
"""
struct DelayedRobot <: AbstractDisturbance
    id::Int
    dt::Int
end

export DroppedObject
"""
    DroppedObject

Object `id` dropped.
"""
struct DroppedObject <: AbstractDisturbance
    id::Int
end


export handle_disturbance!

# function remove_vtxs(sched,remove_set::Set{A}) where {A<:AbstractID}
#     remove_vtxs(sched,Set{Int}(get_vtx(sched,id) for id in remove_set))
# end
function remove_vtxs!(sched,remove_set)
    new_sched = sched
    rem_nodes!(new_sched,remove_set)
    set_leaf_operation_vtxs!(new_sched)
    process_schedule!(new_sched)
    new_sched
end
# function remove_vtxs(sched,remove_set)
#     # Construct new graph
#     # new_sched = sched
#     # rem_nodes!(new_sched,remove_set)
#     # set_leaf_operation_vtxs!(new_sched)
#     # process_schedule!(new_sched)

#     new_sched = OperatingSchedule()
#     keep_vtxs = setdiff(Set{Int}(collect(vertices(sched))), remove_set)
#     # add all non-deleted nodes to new project schedule
#     for v in keep_vtxs
#         add_node!(new_sched,get_node(sched,v))
#     end
#     # add all edges between nodes that still exist
#     for e in edges(get_graph(sched))
#         add_edge!(new_sched, get_vtx_id(sched, e.src), get_vtx_id(sched, e.dst))
#     end
#     set_leaf_operation_vtxs!(new_sched)
#     process_schedule!(new_sched)
#     # @assert sanity_check(new_sched)
#     new_sched
# end

"""
    get_delivery_task_vtxs(sched::OperatingSchedule,o::ObjectID)

Return all vertices that correspond to the delivery task
(`COLLECT` → `CARRY` → `DEPOSIT`) of object `o`.
"""
function get_delivery_task_vtxs(sched::OperatingSchedule,o::ObjectID)
    v = get_vtx(sched,o)
    vtxs = collect(capture_connected_nodes(sched,v,
        v->check_object_id(get_node_from_vtx(sched,v),o)))
    setdiff!(vtxs,v)
    return vtxs
end
"""
    isolate_delivery_task_vtxs(sched,o,vtxs=get_delivery_task_vtxs(sched,o))

Returns:
- incoming: a set of all incoming Action ScheduleNodes
- outgoing: a set of all outgoing Action ScheduleNodes
- op: the target Operation
"""
function isolate_delivery_task_vtxs(sched::OperatingSchedule,o::ObjectID,
        vtxs=get_delivery_task_vtxs(sched,o)
    )
    node_ids = Set{AbstractID}(get_vtx_id(sched,v) for v in vtxs)
    # incoming and outgoing robot nodes -- for single and team robot tasks
    in_edges = exclusive_edge_cover(sched,vtxs,:in)
    incoming = filter(node->matches_template(AbstractRobotAction,node),
        map(e->get_node(sched,e.src),collect(in_edges)))
    out_edges = exclusive_edge_cover(sched,vtxs,:out)
    outgoing = filter(node->matches_template(AbstractRobotAction,node),
        map(e->get_node(sched,e.dst),collect(out_edges)))
    # target operation node
    op = filter(node->matches_template(Operation,node),
        map(e->get_node(sched,e.dst),collect(out_edges)))[1]
    return incoming, outgoing, op
end

"""
    stitch_disjoint_node_sets!(sched,incoming,outgoing)

Finds and adds the appropriate edges between two sets of nodes. It is assumed
that `size(incoming) == size(outgoing)`, that each node in `incoming` has
exactly one feasible successor in `outgoing`, and that each node in `outgoing`
has exactly one feasible predecessor in `incoming`.
"""
function stitch_disjoint_node_sets!(sched,incoming,outgoing,env_state)
    @assert length(incoming) == length(outgoing)
    out_idxs = collect(1:length(outgoing))
    for node1 in incoming
        r = get_robot_id(node1)
        for i in 1:length(outgoing)
            idx = out_idxs[i]
            node2 = outgoing[idx]
            node2 = outgoing[i]
            if get_robot_id(node2) == r
                x = get_location_id(robot_positions(env_state)[r])
                add_edge!(sched,node1,node2)
                n1 = BOT_GO(r,get_initial_location_id(node1),x)
                n2 = BOT_GO(r,x,get_destination_location_id(node2))
                @log_info(-1,0,"connecting ",string(n1)," → ",string(n2))
                replace_in_schedule!(sched,n1,node1.id)
                replace_in_schedule!(sched,n2,node2.id)
                deleteat!(out_idxs,i)
                # deleteat!(outgoing,i)
                break
            end
        end
    end
    @assert length(out_idxs) == 0
    # @assert length(outgoing) == 0
    return sched
end

"""
    handle_disturbance!(solver,prob,env,d::DroppedObject,t,env_state=get_env_state(env,t))

Returns a new `SearchEnv` with a modified `OperatingSchedule`. The new schedule
replaces the previous delivery task (`OBJECT_AT(o,old_x)` → `COLLECT` → `CARRY`
→ `DEPOSIT`) with a new `CleanUpBot` delivery task (`OBJECT_AT(o,new_x)` →
`CUB_COLLECT` → `CUB_CARRY` → `CUB_DEPOSIT`).
It is assumed that the time `t` corresponds to a moment when the object referred
to by `d.id` is being `CARRIED`.
"""
function handle_disturbance!(solver,prob,env::SearchEnv,d::DroppedObject,t,
        env_state = get_env_state(env,t),
        )
    # Schedule surgery
    # - remove Delivery Task: OBJECT_AT(o,old_x) -> COLLECT -> CARRY -> DEPOSIT
    # - replace with equivalent CleanUpBot Delivery Task: OBJECT_AT(o,new_x) -> BOT_COLLECT{CleanUpBot} -> CARRY -> DEPOSIT
    sched = get_schedule(env)
    o = ObjectID(d.id)
    vtxs = get_delivery_task_vtxs(sched,o)
    # incoming and outgoing robot nodes -- for single and team robot tasks
    incoming, outgoing, op = isolate_delivery_task_vtxs(sched,o,vtxs)
    # Add GO->GO edges for affected robot(s), as if they were never assigned
    stitch_disjoint_node_sets!(sched,incoming,outgoing,env_state)
    # remove old nodes
    remove_vtxs!(sched,vtxs)
    # add new CleanUpBot task nodes
    x = get_location_ids(object_position(env_state,o))
    replace_in_schedule!(sched,OBJECT_AT(o,x),o)
    set_t0!(sched,o,t)
    add_headless_delivery_task!(
        sched,get_problem_spec(env,:CleanUpBot),o,op.id,CleanUpBot;
        t0=t
    )
    process_schedule!(sched)
    construct_search_env(
        solver,
        sched,
        env,
        initialize_planning_cache(sched)
    )
end


function apply_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    remove_edges!(env_graph,d.edges)
    return env_graph
end
function remove_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    add_edges!(env_graph,d.edges)
    return env_graph
end

"""
    regenerate_path_specs!(solver,env)

Recompute all paths specs (to account for changes in the env_graphs that will
be propagated to the ProblemSpec's distance function as well.)
"""
function regenerate_path_specs!(solver,env)
    sched = get_schedule(env)
    for v in vertices(sched)
        node = get_node_from_vtx(sched,v)
        prob_spec = get_problem_spec(env,graph_key(node))
        set_path_spec!(sched,v,generate_path_spec(sched,prob_spec,node))
    end
    update_planning_cache!(solver,env)
    return env
end

function add_headless_cleanup_task!(
        sched::OperatingSchedule,
        spec::ProblemSpec,
        d::OilSpill,
        return_vtx::LocationID=LocationID(-1)
        ;
        t0=0,
        )

    robot_id = CleanUpBotID(-1)
    action_id = get_unique_id(ActionID)
    add_to_sched!(sched, spec, CLEAN_UP(robot_id, d.vtxs), action_id)
    set_t0!(sched,action_id,t0)

    prev_action_id = action_id
    action_id = get_unique_id(ActionID)
    add_to_sched!(sched, spec, CUB_GO(robot_id, d.vtxs[1], return_vtx), action_id)
    set_t0!(sched,action_id,t0)
    add_edge!(sched, prev_action_id, action_id)

    return
end

function remove_robot!(env::SearchEnv,id::BotID,t::Int)
    sched = get_schedule(env)
    G = get_graph(sched)
    # schedule
    to_remove = Set{AbstractID}(id)
    v = get_vtx(sched,id)
    for vp in edges(bfs_tree(G,v))
        node = get_node(sched,vp)
        # n_id = get_vtx_id(sched,vp)
        # if isa(n_id,ActionID)
        if matches_template(AbstractRobotAction,node)
            # node = get_node_from_id(sched,n_id)
            # if isa(node,BOT_GO)
            if matches_template(BOT_GO,node)
                # push!(to_remove,n_id)
                push!(to_remove,node_id(node))
            else
                # new_node = replace_robot_id(node,-1)
                replace_in_schedule!(sched,get_problem_spec(env),new_node,n_id)
                new_node = replace_robot_id(node.node,-1)
                replace_in_schedule!(sched,get_problem_spec(env),new_node,node_id(node))
            end
        end
    end
    for n_id in to_remove
        delete_node!(sched,n_id)
    end
    # Verify that the robot is no longer in schedule
    for v in vertices(G)
        n_id = get_vtx_id(sched,v)
        if isa(n_id,ActionID)
            node = get_node_from_id(sched,n_id)
            @assert !(get_id(id) in get_valid_robot_ids(node)) "Robot id $(string(id)) should be wiped from schedule, but is present in $(string(node))"
        end
    end
    # Remap other robots' IDs? TODO refactor OperatinSchedule and RoutePlan so that I don't have to remap ids? i.e., have both uids and idxs?
    # - remove BOT_AT node
    # set assignment ids to -1 with replace_robot_id()
    # if in the middle of CARRY, another robot needs to get the object
end

# Simulation

# select_action(robot,env_state,t) = get_a(get_path_node(robot.path,Int(round(t))))

function step_simulation!(env_state,robots,base_station;
        dt::Float64=1.0,
    )
    actions = []
    for robot in robots
        # robots receive instructions from base station
        # robots observe environment
        # robots communicate to base station
        # robots choose actions 
        # push!(actions,)
    end

    # environment steps forward

    # base station updates global map
end