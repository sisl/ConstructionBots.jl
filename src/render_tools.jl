using MeshCat
using Plots
using LightGraphs, GraphUtils
using GeometryBasics
using LDrawParser
using HierarchicalGeometry
using Colors

import Cairo #, Fontconfig
using GraphPlottingBFS
using Compose

"""
    construct_color_map(model_spec,id_map)

Given the color code in the `SubFileRef`s of model_spec, create a dict mapping
node ids to colors. Should only be applicable to models (not assemblies).
"""
function construct_color_map(model_spec,id_map)
    color_map = Dict{Union{String,AbstractID},AlphaColor}()
    ldraw_color_dict = LDrawParser.get_color_dict() 
    for n in get_nodes(model_spec)
        if isa(node_val(n),SubFileRef)
            color_key = node_val(n).color
            if haskey(id_map,node_id(n))
                id = id_map[node_id(n)]
                if haskey(ldraw_color_dict,color_key) && isa(id,ObjectID)
                    color_map[id] = ldraw_color_dict[color_key]
                    color_map[id_map[id]] = color_map[id]
                end
            end
        end
    end
    return color_map
end

"""
    populate_visualizer!(scene_tree,vis,id_map)

Populate the geometry (and transforms) of a MeshCat visualizer.
"""
function populate_visualizer!(scene_tree,vis;
        color_map=Dict(),
        material_type=nothing,
        wireframe=false,
        kwargs...
    )
    vis_root = vis["root"]
    vis_nodes = Dict{AbstractID,Any}()
    # color_dict = LDrawParser.get_color_dict()
    for v in topological_sort_by_dfs(scene_tree)
        node = get_node(scene_tree,v)
        id = node_id(node)
        if is_root_node(scene_tree,v)
            vis_nodes[id] = vis_root[string(id)]
        else
            p = get_vtx_id(scene_tree,get_parent(scene_tree,v))
            vis_nodes[id] = vis_nodes[p][string(id)]
        end
        vis_node = vis_nodes[id]
        # geometry
        geom = get_base_geom(node)
        if !(geom === nothing)
            filtered_geom = filter(m->!isa(m,GeometryBasics.Line),geom)
            M = GeometryBasics.Mesh(
                coordinates(filtered_geom),
                faces(filtered_geom)
                )
            if !(material_type === nothing)
                mat = material_type(wireframe=wireframe,kwargs...)
                mat.color = get(color_map,id,mat.color)
                setobject!(vis_node,M,mat)
            else
                setobject!(vis_node,M)
            end
        end
        settransform!(vis_node,local_transform(node))
    end
    vis_nodes
end

"""
    update_visualizer!(scene_tree,vis_nodes)

Update the MeshCat transform tree.
"""
function update_visualizer!(scene_tree,vis_nodes,nodes=get_nodes(scene_tree))
    for n in nodes
        settransform!(vis_nodes[node_id(n)],local_transform(n))
    end
    return vis_nodes
end

GraphUtils.get_id(s::String) = s
# Rendering schedule nodes
GraphPlottingBFS._title_string(n::RobotNode)            = "R"
GraphPlottingBFS._title_string(n::ObjectNode)           = "O"
GraphPlottingBFS._title_string(n::AssemblyNode)         = "A"
GraphPlottingBFS._title_string(n::TransportUnitNode)    = "T"

GraphPlottingBFS._title_string(::BuildingStep)          = "B"
GraphPlottingBFS._title_string(::SubFileRef)            = "S"
GraphPlottingBFS._title_string(::SubModelPlan)          = "M"

GraphPlottingBFS._title_string(n::ConstructionBots.EntityConfigPredicate) = _title_string(n.entity)
GraphPlottingBFS._title_string(::ConstructionBots.RobotStart)        = "R"
GraphPlottingBFS._title_string(n::ConstructionBots.ObjectStart)      = "O"
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyStart)     = "M"
GraphPlottingBFS._title_string(::ConstructionBots.AssemblyComplete)  = "M"
GraphPlottingBFS._title_string(::ConstructionBots.OpenBuildStep)     = "oB"
GraphPlottingBFS._title_string(::ConstructionBots.CloseBuildStep)    = "cB"
GraphPlottingBFS._title_string(::ConstructionBots.RobotGo)           = "G"
GraphPlottingBFS._title_string(::ConstructionBots.FormTransportUnit) = "C"
GraphPlottingBFS._title_string(::ConstructionBots.DepositCargo)      = "D"
GraphPlottingBFS._title_string(::ConstructionBots.TransportUnitGo)   = "T"
GraphPlottingBFS._title_string(::ConstructionBots.LiftIntoPlace)     = "L"
GraphPlottingBFS._title_string(::ConstructionBots.ProjectComplete)   = "P"

for op in (
    :(GraphPlottingBFS._node_shape),
    :(GraphPlottingBFS._node_color),
    :(GraphPlottingBFS._title_string)
    )
    @eval $op(n::CustomNode,args...) = $op(node_val(n),args...)
end
# GraphPlottingBFS._node_shape(n::CustomNode,args...) = GraphPlottingBFS._node_shape(node_val(n),args...)
# GraphPlottingBFS._node_color(n::CustomNode,args...) = GraphPlottingBFS._node_color(node_val(n),args...)
# GraphPlottingBFS._title_string(n::CustomNode,args...) = GraphPlottingBFS._title_string(node_val(n),args...)

GraphPlottingBFS._subtitle_string(n::Union{CustomNode,SceneNode}) = "$(get_id(node_id(n)))"

SPACE_GRAY = RGB(0.2,0.2,0.2)
BRIGHT_RED = RGB(0.6,0.0,0.2)
LIGHT_BROWN = RGB(0.6,0.3,0.2)
LIME_GREEN = RGB(0.2,0.6,0.2)
BRIGHT_BLUE = RGB(0.0,0.4,1.0)

GraphPlottingBFS._node_color(::RobotNode)                           = SPACE_GRAY
GraphPlottingBFS._node_color(::ObjectNode)                          = SPACE_GRAY
GraphPlottingBFS._node_color(::AssemblyNode)                        = BRIGHT_BLUE
GraphPlottingBFS._node_color(::TransportUnitNode)                   = LIME_GREEN

GraphPlottingBFS._node_color(::BuildingStep)                        = LIGHT_BROWN
GraphPlottingBFS._node_color(::SubFileRef)                          = BRIGHT_RED
GraphPlottingBFS._node_color(::SubModelPlan)                        = SPACE_GRAY

GraphPlottingBFS._node_color(::ConstructionBots.EntityConfigPredicate) = _node_color(n.entity)
GraphPlottingBFS._node_color(::ConstructionBots.RobotStart)         = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.ObjectStart)        = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.AssemblyStart)      = BRIGHT_RED
GraphPlottingBFS._node_color(::ConstructionBots.AssemblyComplete)   = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.OpenBuildStep)      = LIGHT_BROWN
GraphPlottingBFS._node_color(::ConstructionBots.CloseBuildStep)     = LIGHT_BROWN
GraphPlottingBFS._node_color(::ConstructionBots.ProjectComplete)    = SPACE_GRAY
GraphPlottingBFS._node_color(::ConstructionBots.RobotGo)            = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.FormTransportUnit)  = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.TransportUnitGo)    = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.DepositCargo)       = LIME_GREEN
GraphPlottingBFS._node_color(::ConstructionBots.LiftIntoPlace)      = BRIGHT_BLUE

function GraphPlottingBFS.draw_node(g::AbstractCustomNGraph,v,args...;kwargs...) 
    GraphPlottingBFS.draw_node(get_node(g,v),args...;kwargs...)
end