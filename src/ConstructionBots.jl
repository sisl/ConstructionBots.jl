module ConstructionBots

using Parameters
using StaticArrays
using CoordinateTransformations
# using GeometryBasics
using Rotations
using LightGraphs
using GraphUtils
using LDrawParser
using HierarchicalGeometry
# using CRCBS
# using TaskGraphs

################################################################################
############################ Constructing Model Tree ###########################
################################################################################

# model::MPDModel - model.parts contains raw geometry of all parts
# assembly_tree::AssemblyTree - stored transforms of all parts and submodels
# model_schedule - encodes the partial ordering of assembly operations.

export
    DuplicateIDGenerator,
    duplicate_subtree!,
    construct_assembly_graph

"""
    DuplicateIDGenerator{K}

Generates duplicate IDs.
"""
struct DuplicateIDGenerator{K}
    id_counts::Dict{K,Int}
    id_map::Dict{K,K}
    DuplicateIDGenerator{K}() where {K} = new{K}(
        Dict{K,Int}(),
        Dict{K,K}(),
    )
end
_id_type(::DuplicateIDGenerator{K}) where {K} = K
function (g::DuplicateIDGenerator)(id)
    k = get!(g.id_map,id,id)
    g.id_counts[k] = get(g.id_counts,k,0) + 1
    new_id = string(k,"-",string(g.id_counts[k]))
    g.id_map[new_id] = id
    new_id
end
function duplicate_subtree!(g,old_root,d=:out)
    old_node = get_node(g,old_root)
    new_root = add_node!(g,old_node.val)
    if d == :out
        for v in outneighbors(g,old_root)
            new_v = duplicate_subtree!(g,v,d)
            add_edge!(g,new_root,new_v)
        end
    elseif d == :in
        for v in inneighbors(g,old_root)
            new_v = duplicate_subtree!(g,v,d)
            add_edge!(g,new_v,new_root)
        end
    else
        throw(ErrorException("direction must be :in or :out, can't be $d"))
    end
    return new_root
end
# duplicate_subtree!(g::MPDModelGraph,v) = duplicate_subtree!(g,v,g.id_generator)

"""
    MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}

Graph to represent the modeling operations required to build a LDraw model.
Currently used both as an assembly tree and a "model schedule".
In the model schedule, the final model is the root of the graph, and its
ancestors are the operations building up thereto.
"""
@with_kw struct MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}
    graph       ::DiGraph                   = DiGraph()
    nodes       ::Vector{CustomNode{N,ID}}  = Vector{CustomNode{N,ID}}()
    vtx_map     ::Dict{ID,Int}              = Dict{ID,Int}()
    vtx_ids     ::Vector{ID}                = Vector{ID}() # maps vertex uid to actual graph node
    id_generator::DuplicateIDGenerator{ID}  = DuplicateIDGenerator{ID}()
end
create_node_id(g,v::BuildingStep) = g.id_generator("BuildingStep")
create_node_id(g,v::SubModelPlan) = has_vertex(g,model_name(v)) ? g.id_generator(model_name(v)) : model_name(v)
create_node_id(g,v::SubFileRef)   = g.id_generator(model_name(v))
function GraphUtils.add_node!(g::MPDModelGraph{N,ID},val::N) where {N,ID}
    id = create_node_id(g,val)
    add_node!(g,val,id)
end

"""
    add_build_step!(model_graph,build_step,parent=-1)

add a build step to the model_graph, and add edges from all children of the
parent step to the child.
        [   parent_step   ]
           |           |
        [input] ... [input]
           |           |
        [    build_step   ]
"""
function add_build_step!(model_graph,build_step::BuildingStep,preceding_step=-1)
    node = add_node!(model_graph,build_step)
    for line in build_step.lines
        input = add_node!(model_graph,line)
        add_edge!(model_graph,input,node)
        add_edge!(model_graph,preceding_step,input)
    end
    add_edge!(model_graph,preceding_step,node) # Do I want this or not?
    node
end
function populate_model_subgraph!(model_graph,model::SubModelPlan)
    n = add_node!(model_graph,model)
    preceding_step = -1
    for build_step in model.steps
        preceding_step = add_build_step!(model_graph,build_step,preceding_step)
    end
    add_edge!(model_graph,preceding_step,n)
end

function construct_submodel_dependency_graph(model)
    g = NGraph{DiGraph,SubModelPlan,String}()
    for (k,m) in model.models
        n = add_node!(g,m,k)
    end
    for (k,m) in model.models
        for s in m.steps
            for line in s.lines
                if has_vertex(g,model_name(line))
                    add_edge!(g,model_name(line),k)
                end
            end
        end
    end
    return g
end

"""
Copy all submodel trees into the trees of their parent models.
"""
function copy_submodel_trees!(sched,model)
    sub_model_dependencies = construct_submodel_dependency_graph(model)
    for vp in topological_sort_by_dfs(sub_model_dependencies)
        k = get_vtx_id(sub_model_dependencies,vp)
        @assert has_vertex(sched,k) "SubModelPlan $k isn't in sched, but should be"
        for v in vertices(sched)
            node = get_node(sched,v)
            val = node_val(node)
            if isa(val,SubFileRef)
                if model_name(val) == k
                    sub_model_plan = duplicate_subtree!(sched,k,:in)
                    add_edge!(sched,sub_model_plan,v) # add before instead of replacing
                end
            end
        end
    end
    sched
end

"""
    construct_model_schedule(model)

Edges go forward in time.
"""
function construct_model_schedule(model)
    NODE_VAL_TYPE=Union{SubModelPlan,BuildingStep,SubFileRef}
    sched = MPDModelGraph{NODE_VAL_TYPE,String}()
    for (k,m) in model.models
        populate_model_subgraph!(sched,m)
    end
    copy_submodel_trees!(sched,model)
    return sched
end

"""
    extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}

From a model schedule with (potentially) multiple distinct models, extract just
the model graph with root id `model_key`.
"""
function extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}
    new_sched = S(id_generator=sched.id_generator)
    @assert has_vertex(sched,model_key)
    root = get_vtx(sched,model_key)
    add_node!(new_sched,get_node(sched,root),get_vtx_id(sched,root))
    for edge in edges(reverse(bfs_tree(sched,root;dir=:in)))
        src_id = get_vtx_id(sched,edge.src)
        dst_id = get_vtx_id(sched,edge.dst)
        if !has_vertex(new_sched,src_id)
            transplant!(new_sched,sched,src_id)
        end
        if !has_vertex(new_sched,dst_id)
            transplant!(new_sched,sched,dst_id)
        end
        add_edge!(new_sched,src_id,dst_id)
    end
    new_sched
end

GraphUtils.validate_edge(::SubModelPlan,::SubFileRef) = true
GraphUtils.validate_edge(::BuildingStep,::SubModelPlan) = true
GraphUtils.validate_edge(::BuildingStep,::BuildingStep) = true
GraphUtils.validate_edge(::BuildingStep,::SubFileRef) = true
GraphUtils.validate_edge(::SubFileRef,::BuildingStep) = true

GraphUtils.eligible_successors(::SubFileRef) = Dict(BuildingStep=>1)
GraphUtils.eligible_predecessors(::SubFileRef) = Dict(BuildingStep=>1,SubModelPlan=>1)
GraphUtils.required_successors(::SubFileRef) = Dict(BuildingStep=>1)
GraphUtils.required_predecessors(::SubFileRef) = Dict()

GraphUtils.eligible_successors(::SubModelPlan) = Dict(SubFileRef=>1)
GraphUtils.eligible_predecessors(::SubModelPlan) = Dict(BuildingStep=>1)
GraphUtils.required_successors(::SubModelPlan) = Dict()
GraphUtils.required_predecessors(::SubModelPlan) = Dict(BuildingStep=>1)

GraphUtils.eligible_successors(::BuildingStep) = Dict(SubFileRef=>typemax(Int),SubModelPlan=>1,BuildingStep=>1)
GraphUtils.eligible_predecessors(n::BuildingStep) = Dict(SubFileRef=>LDrawParser.n_lines(n),BuildingStep=>1)
GraphUtils.required_successors(::BuildingStep) = Dict(Union{SubModelPlan,BuildingStep}=>1)
GraphUtils.required_predecessors(n::BuildingStep) = Dict(SubFileRef=>LDrawParser.n_lines(n))

# Write your package code here.
"""
    construct_assembly_graph(model)

Construct an assembly graph, where each `SubModelPlan` has an outgoing edge to
each `SubFileRef` pointing to one of its components.
"""
function construct_assembly_graph(model)
    NODE_VAL_TYPE=Union{SubModelPlan,SubFileRef}
    model_graph = MPDModelGraph{NODE_VAL_TYPE,String}()
    for (k,m) in model.models
        n = add_node!(model_graph,m,k)
        for s in m.steps
            for line in s.lines
                np = add_node!(model_graph,line) #,id_generator(line.file))
                add_edge!(model_graph,n,np)
            end
        end
    end
    return model_graph
end

end
