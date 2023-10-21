"""
    convert_env_graph_to_undirected(G)

It is necessary to convert the env graph to an undirected graph because the
gadget is based on undirected edges. Self-edges also need to be removed, as 
these are already accounted for in the gadget.
"""
function convert_env_graph_to_undirected(G)
    env_graph = Graph(G)
end

# [START]-----------------<_STAY>------------------[MID]--<_BRIDGE>--[START]
#   |                                               |
#    |--<_EDGE>--|                     |--<_EDGE>--|
#              [GADGET]--<_EDGE>--[GADGET]             
#    |--<_EDGE>--|                     |--<_EDGE>--|
#   |                                               |
# [START]-----------------<_STAY>------------------[MID]--<_BRIDGE>--[START]
@enum FlowNodeType begin
    _START  # beginning of a new time step
    _GADGET # inside of gadget layer
    _MID    # end of gadget layer
end

@enum FlowEdgeType begin
    _STAY # wait for a time step
    _BRIDGE # carry over between gadget layers
    _EDGE # edge vtx
end

struct FlowNode
    type::FlowNodeType # type of node
    v::Int # vertex of node in original graph
    t::Int # time step of node
end

struct FlowEdge
    type::FlowEdgeType
    e::Edge
    # t::Int
end

export FlowGraph
"""
    FlowGraph

Represents a time-extended graph useful for MILP formulations. Each vertex of
the `FlowGraph` corresponds to a "flow edge"
"""
const FlowGraph = NEGraph{DiGraph,FlowNode,FlowEdge,VtxID}
GraphUtils.add_node!(graph::FlowGraph,n::FlowNode) = add_node!(graph,n,get_unique_id(VtxID))
get_vtx_from_var(G::FlowGraph,v)      = has_vertex(G,v) ? node_val(get_node(G,v)).v : -1
get_t_from_var(G::FlowGraph,v)        = has_vertex(G,v) ? node_val(get_node(G,v)).t : -1
# is_movement_vtx(graph::FlowGraph,v)   = node_val(get_node(graph,v)).type == _EDGE
# is_stay_vtx(graph::FlowGraph,v)       = node_val(get_node(graph,v)).type == _STAY
# is_bridge_vtx(graph::FlowGraph,v)     = node_val(get_node(graph,v)).type == _BRIDGE
is_movement_edge(graph::FlowGraph,e)  = edge_val(get_edge(graph,e)).type == _EDGE
is_stay_edge(graph::FlowGraph,e)      = edge_val(get_edge(graph,e)).type == _STAY
is_bridge_edge(graph::FlowGraph,e)    = edge_val(get_edge(graph,e)).type == _BRIDGE
add_movement_edge!(graph::FlowGraph,u,v)   = add_edge!(graph,u,v,FlowEdge(_EDGE,   Edge(get_vtx(graph,u),get_vtx(graph,v))))
add_bridge_edge!(graph::FlowGraph,u,v)     = add_edge!(graph,u,v,FlowEdge(_BRIDGE, Edge(get_vtx(graph,u),get_vtx(graph,v))))
add_stay_edge!(graph::FlowGraph,u,v)       = add_edge!(graph,u,v,FlowEdge(_STAY,   Edge(get_vtx(graph,u),get_vtx(graph,v))))

is_start_vtx(graph::FlowGraph,v)      = node_val(get_node(graph,v)).type == _START
is_gadget_vtx(graph::FlowGraph,v)     = node_val(get_node(graph,v)).type == _GADGET
is_mid_vtx(graph::FlowGraph,v)        = node_val(get_node(graph,v)).type == _MID
add_start_vtx!(graph::FlowGraph,v,t)  = add_node!(graph,FlowNode(_START, v,t))
add_gadget_vtx!(graph::FlowGraph,v,t) = add_node!(graph,FlowNode(_GADGET,v,t))
add_mid_vtx!(graph::FlowGraph,v,t)    = add_node!(graph,FlowNode(_MID,   v,t))


"""
    get_source_map(G::FlowGraph,env_graph,TMAX)

Return a source map such that source_map[v][t] points to the corresponding _START
vertex in the gadget graph.
"""
function get_source_map(G::FlowGraph,env_graph,TMAX)
    source_map = [Dict{Int,Int}() for v in 1:nv(env_graph)]
    for idx in vertices(G)
        if is_start_vtx(G,idx) # was is_bridge_vtx
            v = get_vtx_from_var(G,idx)
            t = get_t_from_var(G,idx)
            @assert has_vertex(env_graph,v)
            source_map[v][t] = idx
        end
    end
    return source_map
end

"""
    add_vertex_layer!(G::FlowGraph,incoming::Dict{Int,Vector{Int}},t)

`incoming` points to `_MID` vertices.
        [MID (already added)]--<_BRIDGE>--[START]
"""
function add_vertex_layer!(G::FlowGraph,incoming::Vector{Int},t)
    outgoing = zeros(Int,length(incoming))
    for (v,np) in enumerate(incoming)
        n = add_start_vtx!(G,v,t)
        if has_vertex(G,np)
            add_bridge_edge!(G,np,n)
        end
        outgoing[v] = get_vtx(G,n)
    end
    return outgoing
end

"""
    add_gadget_layer!(G::FlowGraph,edge_list,t,vtx_map=Dict{Int,Int}())

vtx_map points to `_START` vertices

[START (already added)]-----------------<_STAY>------------------[MID]
             |                                               |
              |--<_EDGE>--|                     |--<_EDGE>--|
                        [GADGET]--<_EDGE>--[GADGET]             
              |--<_EDGE>--|                     |--<_EDGE>--|
             |                                               |
[START (already added)]-----------------<_STAY>------------------[MID]
"""
function add_gadget_layer!(G::FlowGraph,edge_list,t,incoming=Vector{Int}())
    outgoing = zeros(Int,length(incoming)) # maps original env vtx to mid layer of flow graph
    for (v,n) in enumerate(incoming)
        np = add_mid_vtx!(G,v,t+1)
        add_stay_edge!(G,n,np)
        outgoing[v] = nv(G)
    end
    for e in edge_list
        if !(e.src == e.dst)
            s1 = incoming[e.src]
            s2 = incoming[e.dst]
            m1 = outgoing[e.src]
            m2 = outgoing[e.dst]
            g1 = add_gadget_vtx!(G,-1,t)
            g2 = add_gadget_vtx!(G,-1,t)
            # gadget edges
            add_movement_edge!(G,s1,g1)
            add_movement_edge!(G,s2,g1)
            add_movement_edge!(G,g1,g2)
            add_movement_edge!(G,g2,m1)
            add_movement_edge!(G,g2,m2)
        end
    end
    return outgoing
end

function construct_gadget_graph(env_graph,TMAX,t0=0,cap=true)
    @assert !is_directed(env_graph)
    G = FlowGraph()
    incoming = zeros(Int,nv(env_graph))
    edge_list = collect(edges(env_graph))
    for t in t0:TMAX-1
        outgoing = add_vertex_layer!(G,incoming,t)
        incoming = add_gadget_layer!(G,edge_list,t,outgoing)
    end
    if cap
        add_vertex_layer!(G,incoming,TMAX)
    end
    return G
end

function _get_layout_coords(G::FlowGraph;dx=1,dt=4,kwargs...)
    x = zeros(nv(G))
    t = zeros(nv(G))
    for v in vertices(G)
        n = node_val(get_node(G,v))
        if TaskGraphs.is_start_vtx(G,v)
            x[v] = dx*n.v
            t[v] = dt*n.t
        elseif TaskGraphs.is_gadget_vtx(G,v)
            if indegree(G,v) == 2
                x[v] = dx*sum(node_val(get_node(G,vp)).v for vp in inneighbors(G,v))/2
                t[v] = dt*(n.t + 1/4)
            else
                x[v] = dx*sum(node_val(get_node(G,vp)).v for vp in outneighbors(G,v))/2
                t[v] = dt*(n.t + 1/2)
            end
        elseif TaskGraphs.is_mid_vtx(G,v)
            x[v] = dx*n.v
            t[v] = dt*(n.t - 1/4)
        end
    end
    x,t
end

# """
#     add_point_constraint!(model,source_map,flow,v,t=0)

# Constrain flow to be equal to 1 at vertex `v`, time `t`
# """
# function add_point_constraint!(model,source_map,flow,v,t=0)
#     idx = source_map[v][t]
#     if outdegree(G,idx) > 0
#         @constraint(model,1 == sum(map(vp->flow[vp],outneighbors(G,idx))))
#     end
#     if indegree(G,idx) > 0
#         @constraint(model,1 == sum(map(vp->flow[vp],inneighbors(G,idx))))
#     end
#     # @constraint(model, flow[idx] == 1)
#     return model
# end
_out_edge_idxs(G,edge_map,v) = [edge_map[Edge(v,vp)] for vp in outneighbors(G,v)]
_in_edge_idxs(G,edge_map,v) = [edge_map[Edge(vp,v)] for vp in inneighbors(G,v)]
"""
    add_source_constraint!(model,source_map,flow,v,t=0)

Constrain flow to be equal to 1 leaving vertex `v`, time `t`
"""
function add_source_constraint!(model,G,source_map,edge_map,flow,v,t=0)
    idx = source_map[v][t]
    if outdegree(G,idx) > 0
        @constraint(model,1 == sum(flow[_out_edge_idxs(G,edge_map,idx)]))
    else
        @warn "vertex $v cannot be a source at time $t--it has no outneighbors"
    end
    return model
end
"""
    add_sink_constraint!(model,source_map,flow,v,t=0)

Constrain flow to be equal to 1 leaving vertex `v`, time `t`
"""
function add_sink_constraint!(model,G,source_map,edge_map,flow,v,t=0)
    idx = source_map[v][t]
    if indegree(G,idx) > 0
        @constraint(model,1 == sum(flow[_in_edge_idxs(G,edge_map,idx)]))
    else
        @warn "vertex $v cannot be a source at time $t--it has no outneighbors"
    end
    return model
end

"""
    add_capacity_constraint!(model,edge_idx)
"""
function add_capacity_constraint!(model,flow,edge_idx,limit=1)
    @constraint(model, flow[edge_idx] <= limit)
end

function add_continuity_constraints!(model,G,edge_map,flow,
    vtx_list=(v for v in vertices(G) if indegree(G,v) > 0 && outdegree(G,v) > 0),
    )
    for v in vtx_list
        in_idxs = _in_edge_idxs(G,edge_map,v)
        out_idxs = _out_edge_idxs(G,edge_map,v)
        @constraint(model, sum(flow[in_idxs]) == sum(flow[out_idxs])) # unit capacity edges
    end
    #     @constraint(model,1 >= sum(map(vp->flow[vp],outneighbors(G,v)))) # unit capacity edges
    #     @constraint(model,1 >= sum(map(vp->flow[vp],inneighbors(G,v)))) # unit capacity edges
    #     if outdegree(G,v) > 0
    #         @constraint(model,
    #             flow[v] <= sum(map(vp->flow[vp],outneighbors(G,v))) # No disappearing robots
    #             )
    #     end
    #     if indegree(G,v) > 0
    #         @constraint(model,
    #             flow[v] <= sum(map(vp->flow[vp],inneighbors(G,v))) # No appearing robots
    #             )
    #     end
    # end
    return model
end
function add_single_transporter_constraint!(model,G,source_map,robot_flow,object_flow,start,goal)
    T = length(source_map[goal])-1
    for (idx,e) in enumerate(edges(G))
        if is_bridge_edge(G,e) || is_stay_edge(G,e)
            vtx = get_vtx_from_var(G,e.src)
            if !(vtx == start || vtx == goal)
                # if object not at start or goal, it cannot be without a robot
                @constraint(model, object_flow[idx] <= robot_flow[idx])
            end
        end
    end
    return model
end
function add_single_object_per_robot_constraints!(model,G,object_flows)
    for (idx,e) in enumerate(edges(G))
        if is_movement_edge(G,e)
            # objects may not overlap on movement vtxs (meaning that robot may not
            # collect multiple objects at once)
            @constraint(model, sum(flow[idx] for flow in values(object_flows)) <= 1)
        end
    end
    return model
end
function add_carrying_constraints!(model,G,robot_flow,object_flows)
    for (idx,e) in enumerate(edges(G))
        if is_movement_edge(G,e)
            # simultaneously constrains object motion (must be on a robot to 
            # move) and robot capacity (only one object at a time). This 
            # constraint therefore subsumes `add_single_object_per_robot_constraints`
            @constraint(model, sum(flow[idx] for flow in values(object_flows)) <= robot_flow[idx])
            # object may not traverse an edge unless it is being carried
            # @constraint(model, object_flow[idx] - robot_flow[idx] <= 0)
        end
    end
    return model
end
function add_precedence_constraints!(model,G,source_map,edge_map,inflow,outflow,goal,start,Δt=0)
    T = length(source_map[goal])-1
    for t in 0:T-(1+Δt) 
        vtx1 = source_map[goal][t]
        vtx2 = source_map[start][t+1+Δt]
        idxs1 = _in_edge_idxs(G,edge_map,vtx1)
        idxs2 = _in_edge_idxs(G,edge_map,vtx2)
        # if inflow is not at vtx1, outflow must still be at vtx2
        @constraint(model,sum(outflow[idxs2]) + sum(inflow[idxs1]) >= 1)
        # @constraint(model,outflow[vtx2] + inflow[vtx1] >= 1)
    end
end
function add_makespan_constraint!(model,T,flow,G,source_map,edge_map,goal,op_duration=0)
    for (t,v) in source_map[goal]
        idxs = _in_edge_idxs(G,edge_map,v)
        @constraint(model,T >= 1+t*(1-sum(flow[idxs]))+op_duration)
        # @constraint(model,T >= 1+t*(1-flow[v])+op_duration)
    end
    return model
end
function add_total_flow_constraint!(model,G,source_map,edge_map,flow,n,t=0)
    # ensure that the sum of the flow across a given time step is equal to n
    edge_idxs = (edge_map[Edge(v,vp)] for v in get_all_root_nodes(G) for vp in outneighbors(G,v))
    @constraint(model,sum(flow[i] for i in edge_idxs) == n)
    # @constraint(model,sum(map(d->flow[d[t]],source_map)) == n)
end

struct PCTAPF_MILP
    model::JuMP.Model
    G::FlowGraph
    source_map::Vector{Dict{Int,Int}}
    edge_map::Dict{Edge,Int}
    robot_flow::Vector{VariableRef}
    object_flows::Dict{ObjectID,Vector{VariableRef}}
end
for op in [
    :(JuMP.optimize!),
    :(JuMP.primal_status),
    :(JuMP.termination_status),
    :(JuMP.objective_bound),
    :(JuMP.objective_value),
    :(JuMP.set_time_limit_sec),
]
    @eval $op(m::PCTAPF_MILP,args...) = $op(m.model,args...)
end
extract_robot_flow(m::PCTAPF_MILP) = Int.(round.(value.(m.robot_flow)))
extract_object_flows(m::PCTAPF_MILP) = Dict(id=>Int.(round.(value.(var))) for (id,var) in m.object_flows)
function extract_flow_path(m::PCTAPF_MILP,flow,v0,t0)
    vtx = m.source_map[v0][0]
    path = Int[vtx]
    while outdegree(m.G,vtx) > 0
        for vp in outneighbors(m.G,vtx)
            # idx = m.edge_map[Edge(vtx,vp)]
            if flow[m.edge_map[Edge(vtx,vp)]] == 1
            # if flow[vp] == 1
                vtx = vp
                break
            end
        end
        push!(path,vtx)
    end
    return path
end
function extract_true_path(m::PCTAPF_MILP,flow_path)
    path = Int[]
    for vtx in flow_path
        if is_start_vtx(m.G,vtx)
        # if is_bridge_vtx(m.G,vtx)
            push!(path,get_vtx_from_var(m.G,vtx))
        end
    end
    return path
end
function extract_robot_paths(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = get_schedule(get_env(prob))
    robot_ICs = get_robot_ICs(sched)
    flow = Int.(round.(value.(m.robot_flow)))
    paths = Dict{RobotID,Vector{Int}}()
    for (id,pred) in robot_ICs
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        flow_path = extract_flow_path(m,flow,v0,t0)
        paths[id] = extract_true_path(m,flow_path)
    end
    return paths
end
function extract_object_paths(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = get_schedule(get_env(prob))
    object_ICs = get_object_ICs(sched)
    paths = Dict{ObjectID,Vector{Int}}()
    for (id,pred) in object_ICs
        flow = Int.(round.(value.(m.object_flows[id])))
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        flow_path = extract_flow_path(m,flow,v0,t0)
        paths[id] = extract_true_path(m,flow_path)
    end
    return paths
end

export BigMILPSolver

@with_kw struct BigMILPSolver <: AbstractPCTAPFSolver
    # EXTRA_T::Int                       = 2
    EXTRA_T::GraphUtils.Counter        = GraphUtils.Counter(2)
    LIMIT_EXTRA_T::Int                 = 64
    direct::Bool                       = false
    logger::SolverLogger{Float64}   = SolverLogger{Float64}()
end

"""
    formulate_big_milp

Formulate a PCTAPF problem as a giant network flow MILP.
"""
function formulate_big_milp(prob::PC_TAPF;
        EXTRA_T=1,
        TMAX = makespan_lower_bound(get_schedule(get_env(prob)),
            get_problem_spec(get_env(prob)))+EXTRA_T,
        direct::Bool = true,
    )
    if direct
        model = direct_model(default_milp_optimizer()()) # reduced memory footprint
    else
        model = JuMP.Model(default_milp_optimizer())
    end
    set_optimizer_attributes(model,default_optimizer_attributes()...)

    sched = get_schedule(get_env(prob))
    route_plan = get_route_plan(get_env(prob))
    robot_ICs = get_robot_ICs(sched)
    object_ICs = get_object_ICs(sched)
    object_starts = Dict{ObjectID,Int}()
    object_goals = Dict{ObjectID,Int}()
    for node in get_nodes(sched)
        if matches_template(BOT_CARRY,node)
            id = get_object_id(node)
            object_starts[id] = get_id(get_initial_location_id(node))
            object_goals[id] = get_id(get_destination_location_id(node))
        end
    end
    env_graph = convert_env_graph_to_undirected(get_graph(get_env(prob)).graph)
    # Choose the time horizon to fix for the flow formulation
    # TMAX = makespan_lower_bound(sched,get_problem_spec(get_env(prob)))+EXTRA_T
    G = construct_gadget_graph(env_graph,TMAX) # for robot flow
    source_map = get_source_map(G,env_graph,TMAX)
    edge_map = Dict(e=>idx for (idx,e) in enumerate(edges(G)))
    # Robot flows
    @variable(model,robot_flow[1:ne(G)], binary=true)
    # Object flows
    object_flows = Dict(id=>@variable(model, [1:ne(G)], binary=true) for id in keys(object_ICs))
    # Flow constraints
    add_continuity_constraints!(model,G,edge_map,robot_flow)
    # Total constraints
    add_total_flow_constraint!(model,G,source_map,edge_map,robot_flow,length(robot_ICs))
    # Carry constraints
    add_carrying_constraints!(model,G,robot_flow,object_flows)
    # add_single_object_per_robot_constraints!(model,G,object_flows)
    # initial constraints
    for (id,pred) in robot_ICs
        v0 = get_id(get_initial_location_id(pred))
        t0 = Int(round(get_t0(get_env(prob),id)))
        add_source_constraint!(model,G,source_map,edge_map,robot_flow,v0,t0)
    end
    # Object constraints
    for (id,pred) in object_ICs
        object_flow = object_flows[id]
        add_total_flow_constraint!(model,G,source_map,edge_map,object_flow,1)
        # initial location
        v0 = get_id(get_initial_location_id(pred))
        t0 = Int(round(get_t0(get_env(prob),id)))
        add_source_constraint!(model,G,source_map,edge_map,object_flow,v0,t0)
        add_continuity_constraints!(model,G,edge_map,object_flow)
        add_single_transporter_constraint!(model,G,source_map,robot_flow,
            object_flow,object_starts[id],object_goals[id])
        add_sink_constraint!(model,G,source_map,edge_map,object_flow,object_goals[id],TMAX)
    end
    # precedence constraints
    for op in values(get_operations(sched))
        for (id2,o2) in postconditions(op)
            outflow = object_flows[id2]
            start = get_id(get_initial_location_id(o2))
            @assert get_id(get_destination_location_id(object_ICs[id2])) == start
            for (id1,o1) in preconditions(op)
                inflow = object_flows[id1]
                goal = get_id(get_destination_location_id(o1))
                # @show string(id1)=>string(id2)
                add_precedence_constraints!(model,G,source_map,edge_map,
                    inflow,outflow,goal,start,duration(op))
            end
        end
    end
    # Objective
    @variable(model,T >= 0) # makespan
    for op in values(get_operations(sched))
        for (id,o) in preconditions(op)
            # makespans
            object_flow = object_flows[id]
            goal = object_goals[id]
            # @show string(o)
            # add_sink_constraint!(model,G,source_map,edge_map,object_flow,goal,TMAX)
            add_makespan_constraint!(model,T,object_flow,G,source_map,edge_map,goal,duration(op))
        end
    end
    @objective(model,Min,T)
    return PCTAPF_MILP(model, G, source_map, edge_map, robot_flow, object_flows)
end


function extract_solution(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = deepcopy(get_schedule(get_env(prob)))
    cache = initialize_planning_cache(sched)
    search_env = SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=get_env(prob).env_layers,
        cost_model=MakeSpan(sched,cache),
        heuristic_model=NullHeuristic())
    route_plan = get_route_plan(search_env)

    robot_paths = extract_robot_paths(prob,m)
    object_paths = extract_object_paths(prob,m)
    # extract route plan
    for (robot_id,robot_path) in robot_paths
        path = get_paths(route_plan)[get_id(robot_id)]
        for (t,(v1,v2)) in enumerate(zip(robot_path[1:end-1],robot_path[2:end]))
            s = State(v1,t-1)
            a = GraphAction(LightGraphs.Edge(v1,v2),1)
            sp = State(v2,t)
            push!(path,PathNode(s,a,sp))
        end
    end
    # extract schedule
    assignments = Set{Tuple{Int,RobotID,ObjectID}}()
    unmatched_object_ids = Set{ObjectID}(keys(object_paths))
    for (object_id,object_path) in object_paths
        object_id in unmatched_object_ids ? nothing : continue
        for (robot_id,robot_path) in robot_paths
            object_id in unmatched_object_ids ? nothing : continue
            for (t,(r_vtx,o_vtx)) in enumerate(zip(robot_path,object_path)) 
                if (o_vtx != object_path[1])
                    if r_vtx == o_vtx 
                        assignment = (t,robot_id,object_id)
                        # @show string(robot_id), string(object_id), r_vtx, o_vtx, assignment
                        push!(assignments,assignment)
                        delete!(unmatched_object_ids, object_id)
                    end
                    # there must be different robot
                    break
                end
            end
        end
    end
    sched = get_schedule(search_env)
    # Add assignments to schedule
    robot_tips = robot_tip_map(sched)
    collect_tips = Dict{ObjectID,ScheduleNode}(
        get_object_id(n) => n for n in get_nodes(sched) if matches_template(BOT_COLLECT,n)
    )
    for assignment in sort(collect(assignments);by=a->a[1])
        t, robot_id, object_id = assignment
        collect_node = collect_tips[object_id]
        robot_node = get_node(sched,robot_tips[robot_id])
        # @show string(robot_node.node)
        # @show string(align_with_successor(robot_node.node,collect_node.node))
        robot_node = replace_in_schedule!(sched,
            get_problem_spec(search_env),
            align_with_successor(robot_node.node,collect_node.node),
            node_id(robot_node)
            )
        # @show string(robot_node.node), robot_node.spec
        add_edge!(sched,robot_node,collect_node)
        propagate_valid_ids!(sched,get_problem_spec(search_env))
        robot_tips = robot_tip_map(sched)
        # robot_tips[robot_id] = new_tip[robot_id]
    end
    # Sync schedule with route plan
    propagate_valid_ids!(sched,get_problem_spec(search_env))
    for node in get_nodes(sched)
        if matches_template(BOT_COLLECT,node)
            object_path = object_paths[get_object_id(node.node)]
            for (t,o_vtx) in zip(Base.Iterators.countfrom(0),object_path)
                if !(o_vtx == object_path[1])
                    set_t0!(node,t-1)
                    break
                end
            end
        elseif matches_template(BOT_DEPOSIT,node)
            object_path = object_paths[get_object_id(node.node)]
            for (t,o_vtx) in zip(Base.Iterators.countfrom(length(object_path)-1,-1),reverse(object_path))
                if !(o_vtx == object_path[end])
                    set_t0!(node,t+1)
                    break
                end
            end
        end
    end
    process_schedule!(sched)
    for node in get_nodes(sched)
        if matches_template(Union{BOT_CARRY,BOT_GO},node)
            if outdegree(sched,node) > 0
                set_tF!(node,get_t0(sched,outneighbors(sched,node)[1]))
            end
        end
    end
    update_cost_model!(search_env)
    set_cost!(search_env,makespan(sched))
    return search_env
end

function CRCBS.solve!(solver::BigMILPSolver,prob::PC_TAPF)
    set_runtime_limit!(solver,max(0,min(runtime_limit(solver),1000)))
    EXTRA_T = get_counter_status(solver.EXTRA_T)
    milp = formulate_big_milp(prob; EXTRA_T=EXTRA_T, direct=solver.direct)
    set_time_limit_sec(milp, runtime_limit(solver))
    optimize!(milp)
    while !(primal_status(milp) == MOI.FEASIBLE_POINT)
        if termination_status(milp) in Set([MOI.TIME_LIMIT, MOI.NODE_LIMIT, MOI.MEMORY_LIMIT, MOI.OTHER_LIMIT])
            @warn "time or node limit reached: status = $(termination_status(milp))"
            break
        end
        @show EXTRA_T = max(EXTRA_T,1) * 2
        if EXTRA_T > solver.LIMIT_EXTRA_T
            break
        end
        milp = formulate_big_milp(prob; EXTRA_T=EXTRA_T, direct=solver.direct)
        set_time_limit_sec(milp, runtime_limit(solver))
        optimize!(milp)
    end
    if primal_status(milp) == MOI.FEASIBLE_POINT
        bound = Int(round(objective_bound(milp)))
        cost = Int(round(objective_value(milp)))
        set_lower_bound!(solver,bound)
        set_best_cost!(solver,cost)
        env = extract_solution(prob,milp)
    else
        cost = Inf
        set_best_cost!(solver,Inf)
        env = deepcopy(get_env(prob))
    end
    return env, cost
end

construct_cost_model(solver::BigMILPSolver,sched,cache,args...;kwargs...) = MakeSpan(sched,cache), NullHeuristic()