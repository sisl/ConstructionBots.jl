export
    FLowProblems

module FlowProblems

# using ..CRCBS
using Graphs, MetaGraphs, JuMP

"""
    `extend_G_through_time(G,T::Int;render_depth=4)`

    Returns a time-extended (to horizon of T steps) version of G.
    Extension is performed via the "gadget" introduced by Yu and LaValle.

"""
function extend_G_through_time(G,T::Int;render_depth=4)
    N = nv(G)
    Gx = MetaDiGraph(nv(G)*(T+1))
    for v in vertices(G)
        for t in 0:T
            set_props!(Gx,v+N*t,Dict(:x=>get_prop(G,v,:x),:y=>render_depth*t))
        end
    end
    for t in 1:T
        edge_list = collect(edges(G))
        for e in edge_list
            x_mid = .5*(get_prop(Gx,e.src,:x)+get_prop(Gx,e.dst,:x))
            # gadget:
            #   o     o
            #   | \ / |
            #   |  o  |
            #   |  |  |
            #   |  o  |
            #   | / \ |
            #   o     o
            #   |     |
            #   o     o
            add_vertex!(Gx,Dict(:x=>x_mid,:y=>render_depth*(t-1) + render_depth*0.25))
            v = nv(Gx)
            add_edge!(Gx,e.src+N*(t-1),v)
            add_edge!(Gx,e.dst+N*(t-1),v)
            add_vertex!(Gx,Dict(:x=>x_mid,:y=>render_depth*(t-1) + render_depth*0.5))
            vp = nv(Gx)
            add_edge!(Gx,v,vp)
            add_vertex!(Gx,Dict(:x=>get_prop(Gx,e.src,:x),:y=>render_depth*(t-1) + render_depth*0.75))
            add_vertex!(Gx,Dict(:x=>get_prop(Gx,e.dst,:x),:y=>render_depth*(t-1) + render_depth*0.75))
            add_edge!(Gx,vp,nv(Gx)-1)
            add_edge!(Gx,vp,nv(Gx))
            # remain at node
            add_edge!(Gx,nv(Gx)-1,e.src+N*t)
            add_edge!(Gx,nv(Gx),e.dst+N*t)
            # skip
            add_edge!(Gx,e.src+N*(t-1),nv(Gx)-1)
            add_edge!(Gx,e.dst+N*(t-1),nv(Gx))
        end
    end
    Gx
end

"""
    `construct_ILP_flow_model(G,T::Int)`

    Constructs a time-extended version of the graph G out to a horizon
    of T steps, then returns both the ILP optimization model and the
    time extended graph Gx
"""
function construct_ILP_flow_model(G,T::Int,starts::Vector{Int},goals::Vector{Int},optimizer;TimeLimit=50, OutputFlag=0)
    Gx = extend_G_through_time(G,T;render_depth=4)
    model = Model(with_optimizer(optimizer,TimeLimit=TimeLimit,OutputFlag=OutputFlag))
    @variable(model, x[1:ne(Gx)], binary=true)
    M = incidence_matrix(Gx)
    srcs_sinks = zeros(Int,nv(Gx),1)
    for v in starts
        srcs_sinks[v] = 1
    end
    for v in goals
        srcs_sinks[v+nv(G)*T] = -1
    end
    # flow constraints between nodes (unit capacity) + sources and - sinks
    @constraint(model, (M*x .+ srcs_sinks) .== 0)
    @objective(model, Min, sum(x))
    return Gx, model, x
end

"""
    `extract_solution_from_flow_matrix(Gx,x)`

    Returns the sequences of vertices in the graph `Gx` that match the
    edge sequences encoded in `x`, the solution matrix of the flow problem
"""
function extract_solution_from_flow_matrix(Gx,x,starts::Vector{Int},goals::Vector{Int})
    edge_list = Vector{Edge}()
    for (i,e) in enumerate(edges(Gx))
        if x[i] == 1
            push!(edge_list,e)
        end
    end
    paths = [[v] for v in starts]
    while length(edge_list) > 0
        for (i,path) in enumerate(paths)
            if length(edge_list) <= 0
                break
            end
            for v in outneighbors(Gx,path[end])
                e = Edge(path[end],v)
                if e in edge_list
                    push!(path,v)
                    setdiff!(edge_list, [e])
                end
            end
        end
    end
    paths
end

"""
    `extract_paths_from_flow_solution(G,paths)`

    Takes the original graph G and the flow solution paths, and returns
    the corresponding edge_sequence through the original graph.
"""
function extract_paths_from_flow_solution(G,paths::Vector{Vector{Int}})
    true_paths = Vector{Vector{Int}}()
    for path in paths
        true_path = Vector{Int}()
        k = 1
        for i in path
            if i <= nv(G)*k
                push!(true_path, i-nv(G)*(k-1))
                k = k + 1
            end
        end
        push!(true_paths, true_path)
    end
    true_paths
end

end
