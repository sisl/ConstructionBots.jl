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
    convert_model_spec_to_abstract_ids(spec,id_map)

Converts model spec to use AbstractIDs for the node ids (rather than strings)
"""

# struct BuildStep

"""
    extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}

From a model schedule with (potentially) multiple distinct models, extract just
the model graph with root id `model_key`.
"""
function extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}
    @assert has_vertex(sched,model_key)
    new_sched = S(id_generator=sched.id_generator)
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
    # for edge in edges(sched)
    #     src_id = get_vtx_id(sched,edge.src)
    #     dst_id = get_vtx_id(sched,edge.dst)
    #     add_edge!(new_sched,src_id,dst_id)
    # end
    new_sched
end

# function add_root_subfile_refs!(model,sched::S) where {S<:MPDModelGraph}
#     for v in get_all_root_nodes(sched)
#         node = get_node(sched,v)
#         val = node_val(node)
#         if isa(val,SubModelPlan)
#             ref = SubFileRef()
#         end
#     end
# end

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
correspondence between ids in different graphs.
"""
function build_id_map(model::MPDModel,spec::MPDModelGraph)
    id_map = Dict{Union{String,AbstractID},Union{String,AbstractID}}()
    for (v,node) in enumerate(get_nodes(spec))
        val = node_val(node)
        new_id = nothing
        if isa(val,SubFileRef)
            if LDrawParser.has_model(model,val.file)
                new_id = get_unique_id(AssemblyID)
            elseif LDrawParser.has_part(model,val.file)
                new_id = get_unique_id(ObjectID)
            else
                continue
            end
        elseif is_terminal_node(spec,v) && isa(val,SubModelPlan)
            # NOTE kind of a hacky way to deal with the root node...
            @info "ADDING ROOT NODE"
            new_id = get_unique_id(AssemblyID)
        end
        if !(new_id === nothing)
            id_map[new_id] = GraphUtils.node_id(node)
            id_map[GraphUtils.node_id(node)] = new_id
        end
    end
    id_map
end

function populate_assembly_subtree!(assembly_tree,spec,id::AssemblyID,id_map)
    node = get_node(assembly_tree,id)
    assembly = node_val(node)
    @assert isa(assembly, AssemblyNode)
    # add edges from Assembly Node to all children
    for e in edges(bfs_tree(spec,id_map[id];dir=:in))
        child_id = get(id_map,get_vtx_id(spec,e.dst),nothing)
        if child_id === nothing
            # @warn "id $(get_vtx_id(spec,e.dst)) missing from id_map"
            continue
        end
        child_ref = node_val(get_node(spec,e.dst))
        @assert has_vertex(assembly_tree,child_id)
        if indegree(assembly_tree,child_id) == 0
            t = LDrawParser.build_transform(child_ref)
            add_component!(assembly,child_id=>t)
            set_child!(assembly_tree,id,child_id)
            @info "$(id_map[id]) => $(get_vtx_id(spec,e.dst)) is $(has_edge(assembly_tree,id,child_id))"
        end
        # if !GraphUtils.validate_embedded_tree(assembly_tree,
        #     v->HierarchicalGeometry.get_transform_node(get_node(assembly_tree,v)),
        #     true # early stop
        #     )
        #     throw(ErrorException("Problem caused when adding $child_id to $assembly"))
        # end
    end
    assembly_tree
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
    for v in topological_sort_by_dfs(spec)
        node = get_node(spec,v)
        if !haskey(id_map,node_id(node))
            continue
        end
        val = node_val(node)
        new_id = id_map[node_id(node)]
        if isa(val,SubModelPlan) # root node
            @info "ROOT: $(node_id(node))"
            g = geom_node(val)
            add_node!(assembly_tree,AssemblyNode(new_id,g),new_id)
            populate_assembly_subtree!(assembly_tree,spec,new_id,id_map)
        elseif isa(val,SubFileRef)
            if has_model(model,val.file)
                @info "SUB FILE ASSEMBLY: $(node_id(node))"
                m = get_model(model,val.file)
                g = geom_node(m)
                add_node!(assembly_tree,AssemblyNode(new_id,g),new_id)
                populate_assembly_subtree!(assembly_tree,spec,new_id,id_map)
            elseif has_part(model,val.file)
                @info "SUB FILE PART: $(node_id(node))"
                p = get_part(model,val.file)
                g = geom_node(p)
                add_node!(assembly_tree,ObjectNode(new_id,g),new_id)
            else
                @warn "SubFileRef points to nonexistent entity $(val.file)"
            end
        elseif isa(val,BuildingStep)
            # do nothing
        end
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

"""
    generate_staging_plan(scene_tree,params)

Given a `SceneTree` representing the final configuration of an assembly, 
construct a plan for the start config, staging config, and final config of
each subassembly and individual object. The idea is to ensure that no node of 
the "plan" graph overlaps with any other. Requires circular bounding geometry
for each component of the assembly. 
"""
function generate_staging_plan(scene_tree,ϵ=0.0,key=HypersphereKey())
    # walk up the tree, computing bounding spheres (cylinders) if necessary
    # SPHERE_TYPE = LazySets.Ball2{Float64,SVector{2,Float64}}
    # spheres = Dict{SceneNodeID,SPHERE_TYPE}()
    for v in reverse(topological_sort_by_dfs(scene_tree))
        node = get_node(scene_tree,v)
        if !has_vertex(node.geom_hierarchy,key)
            if isa(node,ObjectNode)
                add_child_approximation!(node.geom_hierarchy,key,BaseGeomKey())
                # sphere = overapproximate(get_base_geom(node),SPHERE_TYPE,ϵ)
                # spheres[node_id(node)] = sphere
            elseif isa(node,AssemblyNode)
                add_child_approximation!(
                    node.geom_hierarchy,
                    HypersphereKey(),
                    BaseGeomKey(),
                    [t(get_base_geom(get_node(scene_tree,id),key)) for (id,t) in components(node)]
                    )
                # sphere = overapproximation(
                #     [t(spheres[id]) for id in components(node)],
                #     SPHERE_TYPE
                #     )
                # spheres[node_id(node)] = sphere
            end
        end
    end
    # spheres
end



end
