using MeshCat
using Plots
using LightGraphs, GraphUtils
using GeometryBasics
using LDrawParser
using HierarchicalGeometry

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
            id = id_map[node_id(n)]
            if haskey(ldraw_color_dict,color_key) && isa(id,ObjectID)
                color_map[id] = ldraw_color_dict[color_key]
                color_map[id_map[id]] = color_map[id]
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