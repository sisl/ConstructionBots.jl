using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using LightGraphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations
using StaticArrays
using LinearAlgebra

using MeshCat
const MESHCAT_GRID_DIMS = ((-10.0,10.0),(-10.0,10.0))
using Plots

using Logging
global_logger(ConsoleLogger(stderr, Logging.Info))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))

reset_all_id_counters!()

# BASE_ROBOT_HEIGHT   = 0.25
# BASE_ROBOT_RADIUS   = 0.5
MODEL_SCALE         = 0.01
# BASE_ROBOT_SHAPE = get_default_robot_shape(BASE_ROBOT_HEIGHT,BASE_ROBOT_RADIUS)

## LOAD LDRAW FILE
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","Millennium Falcon.mpd")
filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","ATTEWalker.mpd")
model = parse_ldraw_file(filename)
populate_part_geometry!(model)
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),MODEL_SCALE)

## CONSTRUCT MODEL SPEC
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec,"20009 - AT-TE Walker.mpd")
@assert GraphUtils.validate_graph(model_spec)
display_graph(model_spec,scale=1,enforce_visited=true)

## CONSTRUCT SceneTree
id_map = ConstructionBots.build_id_map(model,model_spec)
assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
print(scene_tree,v->"$(summary(node_id(v))) : $(id_map[node_id(v)])","\t")
# Define TransportUnit configurations
ConstructionBots.init_transport_units!(scene_tree;robot_radius = 0.5)
# root = collect(get_all_root_nodes(scene_tree))[1]
# raise assembly so that it is above the x-y plane
# set_local_transform!(scene_tree,root,identity_linear_map() ∘ CoordinateTransformations.Translation(0.0,0.0,0.5))
# capture_child!(scene_tree,AssemblyID(7),AssemblyID(12))
root = get_node(scene_tree,collect(get_all_root_nodes(scene_tree))[1])
validate_tree(HierarchicalGeometry.get_transform_node(root))
validate_embedded_tree(scene_tree,v->HierarchicalGeometry.get_transform_node(get_node(scene_tree,v)))
# @assert(length(get_all_root_nodes(scene_tree)) == 1) 
# visualize
display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned)

# Add robots
robot_geom = GeometryBasics.Cylinder(Point(0.0,0.0,0.0),Point(0.0,0.0,0.25),0.5)
add_node!(scene_tree, RobotNode(RobotID(1),GeomNode(c)))

## Compute overapproximated geometry
HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())

## Construct Partial Schedule
sched = construct_partial_construction_schedule(model,model_spec,scene_tree,id_map)
# Check if schedule graph and embedded transform tree are valid
@assert validate_schedule_transform_tree(sched)
sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,100)
display_graph(sched2,scale=1,enforce_visited=true)
# display_graph(sched,scale=1,enforce_visited=true)

## Generate staging plan
staging_circles = ConstructionBots.generate_staging_plan!(scene_tree,sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

# Select initial Object locations
vtxs = ConstructionBots.construct_vtx_array(;obstacles=collect(values(staging_circles)))
ConstructionBots.select_initial_object_grid_locations!(sched,vtxs)

## Visualize assembly
color_map = construct_color_map(model_spec,id_map)
vis = Visualizer()
render(vis)
delete!(vis)
vis_nodes = populate_visualizer!(scene_tree,vis;
    color_map=color_map,
    material_type=MeshPhongMaterial)
sphere_nodes = show_geometry_layer!(scene_tree,vis_nodes,HypersphereKey())
rect_nodes = show_geometry_layer!(scene_tree,vis_nodes,HyperrectangleKey();
    color=RGBA{Float32}(1, 0, 0, 0.3),
)
staging_nodes = Dict{AbstractID,Any}()
for (id,geom) in staging_circles
    node = get_node(scene_tree,id)
    sphere = Ball2([geom.center..., 0.0],geom.radius)
    setobject!(vis_nodes[id]["staging_circle"],
        convert_to_renderable(sphere),
        MeshPhongMaterial(wireframe=true),
        )
    staging_nodes[id] = vis_nodes[id]["staging_circle"]
end
setvisible!(sphere_nodes,false)
setvisible!(rect_nodes,false)
setvisible!(staging_nodes,false)

# Visualize bounding spheres

# set staging plan and visualize
set_scene_tree_to_initial_condition!(scene_tree,sched)
# restore correct configuration
HG.jump_to_final_configuration!(scene_tree)
# update visualizer
update_visualizer!(scene_tree,vis_nodes)
# Visualize construction
visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;dt=0.1)

# VISUALIZE ROBOT PLACEMENT AROUND PARTS

part_keys = sort(collect(keys(model.parts))[1:10])
parts = Dict(k=>model.parts[k] for k in part_keys)
transport_model = (
    robot_radius = 0.5,
    max_area_per_robot = 100.0, #3000.0,
    max_volume_per_robot = 100.0 #20000.0,
)
for (k,part) in parts
    points = map(SVector{3,Float64}, LDrawParser.extract_points(part))
    if isempty(points)
        println("Part $k has no geometry!")
        continue
    end
    geom=map(SVector,points)
    try
        support_pts = HierarchicalGeometry.select_support_locations(
            geom,transport_model)
        polygon = VPolygon(convex_hull(map(p->p[1:2],geom)))

        plt = plot(polygon,aspectratio=1,alpha=0.4)
        scatter!(plt,map(p->p[1],geom),map(p->p[2],geom),label=k) #,legend=false)
        plot!(plt,map(p->Ball2(p,transport_model.robot_radius),support_pts), aspectratio=1, alpha=0.4)
        display(plt)
    catch e
        bt = catch_backtrace()
        showerror(stdout,e,bt)
    end
end



# # construct model graph
# model_graph = construct_assembly_graph(model)
# model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
# # @assert maximum(map(v->indegree(model_tree,v),vertices(model_tree))) == 1
# print(model_tree,v->summary(v.val),"\t")

# sched = LDrawParser.construct_model_schedule(model)
# model_spec = LDrawParser.extract_single_model(sched,"20009 - AT-TE Walker.mpd")

# GraphUtils.validate_graph(model_spec)
