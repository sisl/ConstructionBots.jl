"""
    The number of edges in each "gadget" per original edge
"""
const GADGET_EDGE_MULTIPLIER = 5
const GADGET_VTX_MULTIPLIER = 2


"""
    It is necessary to convert the env graph to an undirected graph because the
    gadget is based on undirected edges. Self-edges also need to be removed, as
    these are already accounted for in the gadget.
"""
function convert_env_graph_to_undirected(G)
    env_graph = Graph(G)
    # for v in vertices(env_graph)
    #     rem_edge!(env_graph,v,v)
    # end
    # return env_graph
end

"""
    GadgetGraph

Represents a time-extended graph useful for MILP formulations

"""
@with_kw_noshow struct GadgetGraph <: AbstractGraph{Int}
    G::DiGraph          = DiGraph()
    vtxs::Vector{Int}   = Vector{Int}()
    tvec::Vector{Int}   = Vector{Int}()
end
Base.zero(graph::GadgetGraph) = GadgetGraph()
Graphs.edges(graph::GadgetGraph) = edges(graph.G)
Graphs.is_directed(graph::GadgetGraph) = true
function Graphs.add_vertex!(graph::GadgetGraph,t=-1,v=-1)
    if add_vertex!(graph.G)
        push!(graph.vtxs,v)
        push!(graph.tvec,t)
        return true
    end
    return false
end
for op in [
    :edgetype,:has_edge,:has_vertex,:inneighbors,:ne,:nv,:outneighbors,
    :vertices,:add_edge!
    ]
    @eval Graphs.$op(g::GadgetGraph,args...) = $op(g.G,args...)
end
get_vtx_from_var(G::GadgetGraph,v) = get(G.vtxs,v,-1)
get_t_from_var(G::GadgetGraph,v) = get(G.t_vec,v,-1)

"""
    add_movement_vtx!

Flags a vertex of the gadget graph as corresponding to a non-"wait" edge
"""
add_movement_vtx!(graph::GadgetGraph,t) = add_vertex!(graph,t,0)
is_movement_vtx(graph::GadgetGraph,v) = get_vtx_from_var(graph,v) == 0

"""
    Return a source map such that source_map[v][t] points to the corresponding
    vertex in the gadget graph.
"""
function get_source_map(G::GadgetGraph,env_graph,T_MAX)
    # source_map = zeros(Int,nv(env_graph),T_MAX)
    source_map = [Dict{Int,Int}() for v in 1:nv(env_graph)]
    for (idx,v,t) in zip(vertices(G),G.vtxs,G.tvec)
        if has_vertex(env_graph,v)
            source_map[v][t] = idx
        end
    end
    return source_map
end

function add_vertex_layer!(G::GadgetGraph,incoming::Dict{Int,Vector{Int}},t)
    outgoing = Dict{Int,Int}()
    for v in sort(collect(keys(incoming)))
        add_vertex!(G,t,v)
        outgoing[v] = nv(G)
        for v2 in incoming[v]
            add_edge!(G,v2,nv(G))
        end
    end
    return outgoing
end
function add_gadget_layer!(G::GadgetGraph,edge_list,t,vtx_map=Dict{Int,Int}())
    outgoing = Dict{Int,Vector{Int}}()
    for e in edge_list
        if !haskey(vtx_map,e.src) && !haskey(vtx_map,e.dst)
            continue
        end
        if e.src == e.dst
            # shortcut
            add_vertex!(G,t)
            add_edge!(G,vtx_map[e.src],nv(G))
            push!(get!(outgoing,e.dst,Int[]),nv(G))
        else
            if !haskey(vtx_map,e.src)
                e = reverse(e)
            end
            # Gadget
            add_movement_vtx!(G,t)
            v1 = nv(G)
            add_edge!(G,vtx_map[e.src],v1)
            if haskey(vtx_map,e.dst)
                add_movement_vtx!(G,t)
                v2 = nv(G)
                add_edge!(G,vtx_map[e.dst],v2)
            end
            add_movement_vtx!(G,t)
            v3 = nv(G)
            add_edge!(G,v1,v3)
            if haskey(vtx_map,e.dst)
                add_edge!(G,v2,v3)
                add_movement_vtx!(G,t)
                v4 = nv(G)
                add_edge!(G,v3,v4)
                push!(get!(outgoing,e.src,Int[]),v4)
            end
            add_movement_vtx!(G,t)
            v5 = nv(G)
            add_edge!(G,v3,v5)
            push!(get!(outgoing,e.dst,Int[]),v5)
        end
    end
    return outgoing
end

function construct_gadget_graph(env_graph,T_MAX,t0=0,cap=true)
    G = GadgetGraph()
    incoming = Dict(v=>Int[] for v in vertices(env_graph))
    edge_list = collect(edges(env_graph))
    for t in t0:T_MAX-1
        outgoing = add_vertex_layer!(G,incoming,t)
        incoming = add_gadget_layer!(G,edge_list,t,outgoing)
    end
    if cap
        add_vertex_layer!(G,incoming,T_MAX)
    end
    return G
end

function add_point_constraint!(model,source_map,flow,v,t=0)
    idx = source_map[v][t]
    @constraint(model, flow[idx] == 1)
    return model
end
function add_continuity_constraints!(model,G,flow)
    for v in vertices(G)
        @constraint(model,1 >= sum(map(vp->flow[vp],outneighbors(G,v))))
        @constraint(model,1 >= sum(map(vp->flow[vp],inneighbors(G,v))))
        if outdegree(G,v) > 0
            @constraint(model,
                flow[v] <= sum(map(vp->flow[vp],outneighbors(G,v)))
                )
        end
        if indegree(G,v) > 0
            @constraint(model,
                flow[v] <= sum(map(vp->flow[vp],inneighbors(G,v)))
                )
        end
    end
    return model
end
function add_carrying_constraints!(model,G,robot_flow,object_flow)
    for v in vertices(G)
        if is_movement_vtx(G,v)
            # object may not traverse an edge unless it is being carried
            @constraint(model, object_flow[v] - robot_flow[v] <= 0)
        end
    end
    return model
end
function add_precedence_constraints!(model,source_map,inflow,outflow,goal,start,Δt=0)
    for t in 0:size(source_map,2)-Δt
        vtx1 = source_map[goal][t]
        vtx2 = source_map[start][t+1+Δt]
        # if flow1 is not at vtx1, flow2 must still be at vtx2
        @constraint(model,outflow[vtx2] + inflow[vtx1] >= 1)
    end
end
function add_makespan_constraint!(model,T,flow,source_map,goal)
    for (t,v) in source_map[goal]
        @constraint(model,T >= t*(1-flow[v]))
    end
    return model
end
function add_total_flow_constraint!(model,source_map,flow,n,t=0)
    @constraint(model,sum(map(d->flow[d[t]],source_map)) == n)
end

struct PCTAPF_MILP
    model::JuMP.Model
    G::GadgetGraph
    source_map::Vector{Dict{Int,Int}}
    robot_flow::Vector{VariableRef}
    object_flows::Dict{ObjectID,Vector{VariableRef}}
end
JuMP.optimize!(m::PCTAPF_MILP) = JuMP.optimize!(m.model)
extract_robot_flow(m::PCTAPF_MILP) = Int.(round.(value.(m.robot_flow)))
extract_object_flows(m::PCTAPF_MILP) = Dict(id=>Int.(round.(value.(var))) for (id,var) in m.object_flows)
function extract_flow_path(m::PCTAPF_MILP,flow,v0,t0)
    vtx = m.source_map[v0][0]
    path = Int[vtx]
    while outdegree(m.G,vtx) > 0
        for vp in outneighbors(m.G,vtx)
            if flow[vp] == 1
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
        if get_vtx_from_var(m.G,vtx) > 0
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

"""
    formulate_big_milp

Formulate a PCTAPF problem as a giant network flow MILP.
"""
function formulate_big_milp(prob::PC_TAPF,T_MAX,model = JuMP.Model())
    sched = get_schedule(get_env(prob))
    robot_ICs = get_robot_ICs(sched)
    object_ICs = get_object_ICs(sched)
    env_graph = convert_env_graph_to_undirected(get_graph(get_env(prob)).graph)
    G = construct_gadget_graph(env_graph,T_MAX) # for robot flow
    source_map = get_source_map(G,env_graph,T_MAX)
    # Robot flows
    @variable(model,robot_flow[1:nv(G)], binary=true)
    # Object flows
    object_flows = Dict(id=>@variable(model,[1:nv(G)], binary=true) for (id,pred) in object_ICs)
    # Flow constraints
    add_continuity_constraints!(model,G,robot_flow)
    # initial constraints
    for (id,pred) in robot_ICs
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        add_point_constraint!(model,source_map,robot_flow,v0,t0)
    end
    add_total_flow_constraint!(model,source_map,robot_flow,length(robot_ICs))
    # makespan
    @variable(model,T >= 0)
    for (id,pred) in object_ICs
        object_flow = object_flows[id]
        add_total_flow_constraint!(model,source_map,object_flow,1)
        # initial location
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        add_point_constraint!(model,source_map,object_flow,v0,t0)
        add_continuity_constraints!(model,G,object_flow)
        add_carrying_constraints!(model,G,robot_flow,object_flow)
    end
    # precedence constraints
    for op in values(get_operations(sched))
        Δt = duration(op)
        for (id1,o2) in postconditions(op)
            outflow = object_flows[id2]
            start = get_id(get_initial_location_id(o2))
            @assert get_id(get_destination_location_id(object_ICs[id2])) == start
            for (id1,o1) in preconditions(op)
                # makespans
                # id1 = get_object_id(o1)
                inflow = object_flows[id1]
                goal = get_id(get_destination_location_id(o1))
                # @assert get_id(get_destination_location_id(get_object_FCs(sched)[id1])) == goal
                add_precedence_constraints!(model,source_map,inflow,outflow,goal,start,Δt)
            end
        end
    end
    # Objective
    for op in values(get_operations(sched))
        for (id,o) in preconditions(op)
            # makespans
            # id = get_object_id(o)
            object_flow = object_flows[id]
            goal = get_id(get_destination_location_id(o))
            @show o, goal
            add_point_constraint!(model,source_map,object_flow,goal,T_MAX)
            add_makespan_constraint!(model,T,object_flow,source_map,goal)
        end
    end
    @objective(model,Min,T)
    return PCTAPF_MILP(model, G, source_map, robot_flow, object_flows)
end
