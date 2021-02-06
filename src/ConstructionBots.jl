module ConstructionBots
#=  =#
using Parameters
using StaticArrays
using CoordinateTransformations
# using GeometryBasics
using Rotations
using LightGraphs
using GraphUtils
using LDrawParser
using HierarchicalGeometry
using JuMP
using ECOS
using MathOptInterface
using LinearAlgebra

# using CRCBS
# using TaskGraphs
using Logging

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
    BuildStepID <: AbstractID
"""
@with_kw struct BuildStepID <: AbstractID
    id::Int
end
"""
    SubModelPlanID <: AbstractID
"""
@with_kw struct SubModelPlanID <: AbstractID
    id::Int
end
"""
    SubFileRedID <: AbstractID
"""
@with_kw struct SubFileRefID <: AbstractID
    id::Int
end

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
GraphUtils._id_type(::DuplicateIDGenerator{K}) where {K} = K
function (g::DuplicateIDGenerator)(id)
    k = get!(g.id_map,id,id)
    g.id_counts[k] = get(g.id_counts,k,0) + 1
    new_id = string(k,"-",string(g.id_counts[k]))
    g.id_map[new_id] = id
    new_id
end
function duplicate_subtree!(g,old_root,d=:out)
    vtxs = [get_vtx(g,old_root)]
    append!(vtxs, collect(map(e->e.dst,edges(bfs_tree(g,old_root;dir=d)))))
    new_ids = Dict{Int,GraphUtils._id_type(g)}()
    for v in vtxs
        old_node = get_node(g,v)
        new_node = add_node!(g,node_val(old_node))
        new_ids[v] = node_id(new_node)
    end
    f = d == :out ? outneighbors : inneighbors
    for v in vtxs
        for vp in f(g,v)
            if d == :out
                add_edge!(g, new_ids[v], new_ids[vp])
            else
                add_edge!(g, new_ids[vp], new_ids[v])
            end
        end
    end
    return get_node(g,new_ids[get_vtx(g,old_root)])
end

"""
    MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}

Graph to represent the modeling operations required to build a LDraw model.
Currently used both as an assembly tree and a "model schedule".
In the model schedule, the final model is the root of the graph, and its
ancestors are the operations building up thereto.
"""
@with_kw_noshow struct MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}
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

# function add_subfile_reference!(model_graph,ref::SubFileRef)
# end

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
    # if is_root_node(model_graph,node) 
    # if has_vertex(model_graph,preceding_step) && is_terminal_node(model_graph,preceding_step) 
        add_edge!(model_graph,preceding_step,node) # Do I want this or not?
    # end
    node
end
function populate_model_subgraph!(model_graph,model::SubModelPlan)
    n = add_node!(model_graph,model)
    preceding_step = -1
    for build_step in model.steps
        if !isempty(build_step.lines)
            preceding_step = add_build_step!(model_graph,build_step,preceding_step)
        end
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
    update_build_step_parents!(model_spec)

Ensure that all `BuildingStep` nodes have the correct parent (submodel) name.
"""
function update_build_step_parents!(model_spec)
    descendant_map = backup_descendants(model_spec,
        n->matches_template(SubModelPlan,n))
    for v in vertices(model_spec)
        node = get_node(model_spec,v)
        if matches_template(BuildingStep,node)
            node = replace_node!(model_spec,
                BuildingStep(node_val(node),descendant_map[node_id(node)]),
                node_id(node)
                )
            @assert node_val(node).parent == descendant_map[node_id(node)]
        end
    end
    model_spec
end

"""
    construct_model_spec(model)

Edges go forward in time.
"""
function construct_model_spec(model)
    NODE_VAL_TYPE=Union{SubModelPlan,BuildingStep,SubFileRef}
    spec = MPDModelGraph{NODE_VAL_TYPE,String}()
    for (k,m) in model.models
        populate_model_subgraph!(spec,m)
    end
    copy_submodel_trees!(spec,model)
    update_build_step_parents!(spec)
    return spec 
end


"""
    extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}

From a model schedule with (potentially) multiple distinct models, extract just
the model graph with root id `model_key`.
"""
function extract_single_model(spec::S,model_key) where {S<:MPDModelGraph}
    @assert has_vertex(spec,model_key)
    new_spec = S(id_generator=spec.id_generator)
    root = get_vtx(spec,model_key)
    add_node!(new_spec,get_node(spec,root),get_vtx_id(spec,root))
    for v in reverse(topological_sort_by_dfs(spec))
        dst_id = get_vtx_id(spec,v)
        if !has_vertex(new_spec,dst_id)
            continue
        end
        # for v2 in inneighbors(spec,id)
            
        #     if !has_vertex(new_spec,)
        # end
    # for edge in edges(reverse(bfs_tree(spec,root;dir=:in)))
        # TODO bfs_tree is the problem here
        # src_id = get_vtx_id(spec,edge.src)
        if !has_vertex(new_spec,dst_id)
            transplant!(new_spec,spec,dst_id)
        end
        for v2 in inneighbors(spec,dst_id)
            src_id = get_vtx_id(spec,v2)
            if !has_vertex(new_spec,src_id)
                transplant!(new_spec,spec,src_id)
            end
            add_edge!(new_spec,src_id,dst_id)
        end

        # dst_id = get_vtx_id(spec,edge.dst)
        # if !has_vertex(new_spec,dst_id)
        #     transplant!(new_spec,spec,dst_id)
        # end
        # add_edge!(new_spec,src_id,dst_id)
    end
    # for edge in edges(spec)
    #     src_id = get_vtx_id(spec,edge.src)
    #     dst_id = get_vtx_id(spec,edge.dst)
    #     add_edge!(new_spec,src_id,dst_id)
    # end
    new_spec
end

# Edges for Project Spec. TODO dispatch on graph type
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
    # copy_submodel_trees!(model_graph,model)
    return model_graph
end

geom_node(m::DATModel) = GeomNode(LDrawParser.extract_geometry(m))
geom_node(m::SubModelPlan) = GeomNode(nothing)
function geom_node(model::MPDModel,ref::SubFileRef)
    if has_model(model,ref.file)
        return geom_node(get_model(model,ref.file))
    elseif has_part(model,ref.file)
        return geom_node(get_part(model,ref.file))
    end
    throw(ErrorException("Referenced file $(ref.file) is not in model"))
    # @warn "Referenced file $(ref.file) is not in model"
    GeomNode(nothing)
end

"""
    build_id_map(model::MPDModel,spec::MPDModelGraph)

Constructs a `Dict` mapping from `AbstractID <=> String` to keep track of the
correspondence between ids in different graphs. Only the following id types are
mapped:
- ObjectID <=> SubFileRef (if the ref points to a model, not a part)
- AssemblyID <=> SubModelPlan
"""
function build_id_map(model::MPDModel,spec::MPDModelGraph)
    id_map = Dict{Union{String,AbstractID},Union{String,AbstractID}}()
    for (v,node) in enumerate(get_nodes(spec))
        val = node_val(node)
        new_id = nothing
        if isa(val,SubFileRef)
            if LDrawParser.has_model(model,val.file)
                # new_id = get_unique_id(AssemblyID)
            elseif LDrawParser.has_part(model,val.file)
                new_id = get_unique_id(ObjectID)
            else
                continue
            end
        elseif isa(val,SubModelPlan)
            new_id = get_unique_id(AssemblyID)
        end
        if !(new_id === nothing)
            id_map[new_id] = GraphUtils.node_id(node)
            id_map[GraphUtils.node_id(node)] = new_id
        end
    end
    id_map
end

"""
    get_referenced_component(model_spec,scene_tree,id_map,node)

A hacky utility for retrieving the component added to an assembly by 
node::CustomNode{SubFileRef,...}. Currently necessary because some 
SubFileRef nodes reference an object (id_map[object_id] <=> id_map[ref_id]), 
whereas other SubFileRef nodes reference the assembly encoded by their direct parent.
"""
function get_referenced_component(model_spec,id_map,node)
    @assert matches_template(SubFileRef,node)
    # node = get_node(model_spec,id)
    for v in inneighbors(model_spec,node)
        input = get_node(model_spec,v)
        if matches_template(SubModelPlan,input)
            return id_map[get_vtx_id(model_spec,v)]
        end
    end
    return get(id_map,node_id(node),nothing)
end

"""
    get_build_step_components(model_spec,id_map,step)

Given step::CustomNode{BuildingStep,...}, return the set of AbstractIDs pointing
to all of the components to be added to the parent assembly at that building 
step.
"""
function get_build_step_components(model_spec,id_map,step)
    @assert matches_template(BuildingStep,step)
    part_ids = Set{Union{AssemblyID,ObjectID}}()
    for v in inneighbors(model_spec,step)
        child = get_node(model_spec,v)
        if matches_template(SubFileRef,child)
            push!(part_ids,
                get_referenced_component(model_spec,id_map,child))
        end
    end
    return part_ids
end

"""
    construct_assembly_tree(model::MPDModel,spec::MPDModelGraph,

Construct an assembly_tree::NTree{SceneNode,AbstractID}, a precursor to 
SceneTree.
"""
function construct_assembly_tree(model::MPDModel,spec::MPDModelGraph,
        id_map = build_id_map(model,spec),
    )
    assembly_tree = NTree{SceneNode,AbstractID}()
    parent_map = backup_descendants(spec,n->matches_template(SubModelPlan,n))
    for v in reverse(topological_sort_by_dfs(spec))
        node = get_node(spec,v)
        id = node_id(node)
        haskey(id_map,id) ? nothing : continue
        new_id = id_map[id]
        val = node_val(node)
        ref = nothing
        parent_id = id_map[parent_map[id]]
        if isa(val,SubModelPlan)
            g = geom_node(val)
            add_node!(assembly_tree,AssemblyNode(new_id,g),new_id)
            is_terminal_node(spec, v) ? continue : nothing
            # retrieve parent SubFileRef
            ref_node = get_node(spec,outneighbors(spec,v)[1])
            ref = node_val(ref_node)
            @assert isa(ref,SubFileRef) "ref is $(ref)"
            parent_id = id_map[parent_map[node_id(ref_node)]]
        elseif isa(val,SubFileRef)
            if has_model(model,val.file)
                continue # Don't add assembly here
            else has_part(model,val.file) 
                # Adding an object only
                @info "SUB FILE PART: $(node_id(node))"
                p = get_part(model,val.file)
                g = geom_node(p)
                add_node!(assembly_tree,ObjectNode(new_id,g),new_id)
                ref = val
            end
        elseif isa(val,BuildingStep)
            continue
        end
        @info "Attaching $(id_map[parent_id]) => $(id_map[new_id])"
        parent = node_val(get_node(assembly_tree,parent_id))
        t = LDrawParser.build_transform(ref)
        add_component!(parent,new_id=>t)
        set_child!(assembly_tree,parent_id,new_id)
        @info "$(id_map[parent_id]) => $(id_map[new_id]) is $(has_edge(assembly_tree,parent_id,new_id))"
    end
    assembly_tree
end

"""
    convert_to_scene_tree(assembly_tree,set_children=true)

Convert an assembly tree to a `SceneTree`.
"""
function convert_to_scene_tree(assembly_tree;set_children::Bool=true)
    scene_tree = SceneTree()
    for n in get_nodes(assembly_tree)
        add_node!(scene_tree,node_val(n))
    end
    if set_children
        for e in edges(assembly_tree)
            src_id = get_vtx_id(assembly_tree,edge_source(e))
            dst_id = get_vtx_id(assembly_tree,edge_target(e))
            # set_child! will ensure that the full tree is correctly set up.
            set_child!(scene_tree,src_id,dst_id)
        end
    end
    return scene_tree
end

include("construction_schedule.jl")
include("utils.jl")
include("rvo_interface.jl")

### RVO

end
