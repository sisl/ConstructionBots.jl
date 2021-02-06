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
    for v in topological_sort_by_dfs(scene_tree)
        node = get_node(scene_tree,v)
        id = node_id(node)
        # if is_root_node(scene_tree,v)
        #     vis_nodes[id] = vis_root[string(id)]
        # else
        #     p = get_vtx_id(scene_tree,get_parent(scene_tree,v))
        #     vis_nodes[id] = vis_nodes[p][string(id)]
        # end
        vis_nodes[id] = vis_root[string(id)]
        vis_node = vis_nodes[id]
        # geometry
        geom = get_base_geom(node)
        if !(geom === nothing)
            if isa(geom,GeometryPrimitive)
                M = geom
            else
                filtered_geom = filter(m->!isa(m,GeometryBasics.Line),geom)
                M = GeometryBasics.Mesh(
                    coordinates(filtered_geom),
                    faces(filtered_geom)
                )
            end
            if !(material_type === nothing)
                mat = material_type(wireframe=wireframe,kwargs...)
                mat.color = get(color_map,id,mat.color)
                setobject!(vis_node,M,mat)
            else
                setobject!(vis_node,M)
            end
        end
        # settransform!(vis_node,local_transform(node))
        settransform!(vis_node,global_transform(node))
    end
    vis_nodes
end

convert_to_renderable(geom::Ball2) = GeometryBasics.Sphere(geom)
convert_to_renderable(geom::Hyperrectangle) = GeometryBasics.HyperRectangle(geom)

function show_geometry_layer!(scene_tree,vis_nodes,key=HypersphereKey();
        color=RGBA{Float32}(0, 1, 0, 0.3),
        wireframe=true,
        material=MeshPhongMaterial(color=color,wireframe=wireframe),
    )
    geom_nodes = Dict{AbstractID,Any}()
    for (id,vis_node) in vis_nodes
        node = get_node(scene_tree,id)
        geom = get_base_geom(node,key)
        node_name = string(key)
        setobject!(vis_node[node_name],convert_to_renderable(geom),material)
        geom_nodes[id] = vis_node[node_name]
    end
    geom_nodes
end

function MeshCat.setvisible!(vis_nodes::Dict{AbstractID,Any},val)
    for (k,v) in vis_nodes
        setvisible!(v,val)
    end
end

"""
    update_visualizer!(scene_tree,vis_nodes)

Update the MeshCat transform tree.
"""
function update_visualizer!(scene_tree,vis_nodes,nodes=get_nodes(scene_tree))
    for n in nodes
        # settransform!(vis_nodes[node_id(n)],local_transform(n))
        settransform!(vis_nodes[node_id(n)],global_transform(n))
    end
    return vis_nodes
end

function call_update!(scene_tree,vis_nodes,nodes,dt)
    update_visualizer!(scene_tree,vis_nodes,nodes)
    render(vis)
    sleep(dt)
end

function visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;
    dt=0.2,
    )
    for v in topological_sort_by_dfs(sched)
        update = true
        node = get_node(sched,v)
        update_nodes = []
        if matches_template(ProjectComplete,node)
        # elseif matches_template(RobotStart,node)
        #     robot_node = entity(node)
        #     @assert has_parent(robot_node,robot_node)
        # elseif matches_template(ObjectStart,node)
        #     part_node = get_node(scene_tree,node_id(entity(node)))
        #     @assert has_parent(part_node,part_node)
        #     set_local_transform!(part_node,local_transform(goal_config(node)))
        elseif matches_template(RobotGo,node)
            robot_node = entity(node)
            @assert has_parent(robot_node,robot_node)
            set_local_transform!(robot_node,global_transform(goal_config(node)))
            push!(update_nodes,robot_node)
        elseif matches_template(FormTransportUnit,node)
            transport_unit = entity(node)
            part_node = get_node(scene_tree,cargo_id(transport_unit))
            @assert has_parent(part_node,part_node)
            @assert has_parent(transport_unit,transport_unit)
            set_local_transform!(transport_unit,global_transform(goal_config(node)))
            set_child!(scene_tree,transport_unit,part_node)
            @assert has_parent(part_node,transport_unit)
            append!(update_nodes,[transport_unit,part_node])
        elseif matches_template(TransportUnitGo,node)
            transport_unit = entity(node)
            set_local_transform!(transport_unit,global_transform(goal_config(node)))
            append!(update_nodes,[transport_unit])
        elseif matches_template(DepositCargo,node)
            transport_unit = entity(node)
            part_node = get_node(scene_tree,cargo_id(transport_unit))
            @assert has_parent(part_node,transport_unit)
            disband!(scene_tree,transport_unit)
            @assert has_parent(part_node,part_node)
            set_local_transform!(part_node,global_transform(cargo_deployed_config(node)))
            append!(update_nodes,[transport_unit,part_node])
        elseif matches_template(LiftIntoPlace,node)
            part_node = get_node(scene_tree,node_id(entity(node)))
            @assert has_parent(part_node,part_node)
            set_local_transform!(part_node,global_transform(goal_config(node)))
            if matches_template(AssemblyNode,part_node)
                @info "$(string(node_id(node)))" part_node
            end
            append!(update_nodes,[part_node])
        elseif matches_template(CloseBuildStep,node)
            for (id,_) in assembly_components(node)
                set_child!(scene_tree,ConstructionBots.get_assembly(node),id)
                push!(update_nodes,get_node(scene_tree,id))
            end
        end
        if !isempty(update_nodes)
            # id = node_id(node)
            # ids = map(n->string(node_id(n))=>string(global_transform(n).translation), update_nodes)
            # @info "Updating nodes" id ids
            # call_update!(scene_tree,vis_nodes,update_nodes,dt)
            update_visualizer!(scene_tree,vis_nodes)
            render(vis)
            sleep(dt)
        end
    end
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
GraphPlottingBFS._title_string(::ConstructionBots.FormTransportUnit) = "F"
GraphPlottingBFS._title_string(::ConstructionBots.DepositCargo)      = "D"
GraphPlottingBFS._title_string(::ConstructionBots.TransportUnitGo)   = "T"
GraphPlottingBFS._title_string(::ConstructionBots.LiftIntoPlace)     = "L"
GraphPlottingBFS._title_string(::ConstructionBots.ProjectComplete)   = "P"

for op in (
    :(GraphPlottingBFS._node_shape),
    :(GraphPlottingBFS._node_color),
    :(GraphPlottingBFS._title_string),
    :(GraphPlottingBFS._subtitle_string)
    )
    @eval $op(n::CustomNode,args...) = $op(node_val(n),args...)
    @eval $op(n::ScheduleNode,args...) = $op(n.node,args...)
end
# GraphPlottingBFS._node_shape(n::CustomNode,args...) = GraphPlottingBFS._node_shape(node_val(n),args...)
# GraphPlottingBFS._node_color(n::CustomNode,args...) = GraphPlottingBFS._node_color(node_val(n),args...)
# GraphPlottingBFS._title_string(n::CustomNode,args...) = GraphPlottingBFS._title_string(node_val(n),args...)

GraphPlottingBFS._subtitle_string(n::SceneNode) = "$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::Union{EntityGo,EntityConfigPredicate,FormTransportUnit,DepositCargo}) = GraphPlottingBFS._subtitle_string(entity(n))
GraphPlottingBFS._subtitle_string(n::BuildPhasePredicate) = GraphPlottingBFS._subtitle_string(n.assembly)
GraphPlottingBFS._subtitle_string(n::ObjectNode) = "o$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::RobotNode) = "r$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::AssemblyNode) = "a$(get_id(node_id(n)))"
GraphPlottingBFS._subtitle_string(n::TransportUnitNode) = "t$(get_id(node_id(n)))"

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
GraphPlottingBFS._node_color(::ConstructionBots.AssemblyStart)      = SPACE_GRAY
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