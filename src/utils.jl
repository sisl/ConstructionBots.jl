function extract_small_sched_for_plotting(sched,LIM=100)
    sched2 = typeof(sched)()
    frontier = collect(get_all_terminal_nodes(sched))
    explored = Set{Int}()
    while length(explored) < LIM && !isempty(frontier)
        v = popfirst!(frontier)
        if !has_vertex(sched2,get_vtx_id(sched,v))
            add_node!(sched2,get_node(sched,v),get_vtx_id(sched,v))
            push!(explored,v)
            for vp in inneighbors(sched,v)
                if !(vp in explored)
                    push!(frontier,vp)
                end
            end
        end
    end
    for n in get_nodes(sched2)
        for v in outneighbors(sched,node_id(n))
            add_edge!(sched2,n,get_vtx_id(sched,v))
        end
    end
    return sched2
end