using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using LightGraphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations

using MeshCat
using Plots

using Logging
global_logger(SimpleLogger(stderr, Logging.Debug))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))

reset_all_id_counters!()

## LOAD LDRAW FILE
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","Millennium Falcon.mpd")
filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","ATTEWalker.mpd")
model = parse_ldraw_file(filename)
populate_part_geometry!(model)
SCALE = 0.01
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),SCALE)

## CONSTRUCT MODEL SPEC
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec,"20009 - AT-TE Walker.mpd")
@assert GraphUtils.validate_graph(model_spec)

## CONSTRUCT SceneTree
id_map = ConstructionBots.build_id_map(model,model_spec)
assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
# root = collect(get_all_root_nodes(scene_tree))[1]
# raise assembly so that it is above the x-y plane
# set_local_transform!(scene_tree,root,identity_linear_map() ∘ CoordinateTransformations.Translation(0.0,0.0,0.5))
# capture_child!(scene_tree,AssemblyID(7),AssemblyID(12))
print(scene_tree,v->"$(summary(node_id(v))) : $(id_map[node_id(v)])","\t")

root = get_node(scene_tree,collect(get_all_root_nodes(scene_tree))[1])
validate_tree(HierarchicalGeometry.get_transform_node(root))
validate_embedded_tree(scene_tree,v->HierarchicalGeometry.get_transform_node(get_node(scene_tree,v)))


color_map = construct_color_map(model_spec,id_map)

vis = Visualizer()
render(vis)
delete!(vis)
vis_nodes = populate_visualizer!(scene_tree,vis;
    color_map=color_map,
    material_type=MeshPhongMaterial)

n = get_node(scene_tree,1)
geom = get_base_geom(n)
hmodel = equatorial_overapprox_model()
hpoly = LazySets.overapproximate(geom,hmodel)
vpoly = convert(VPolytope,hpoly)

g = GeometryHierarchy()
construct_geometry_tree!(g,geom)

# ConstructionBots.update_build_step_parents!(model_spec)
# model_graph = construct_assembly_graph(model)
# model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
# # @assert maximum(map(v->indegree(model_tree,v),vertices(model_tree))) == 1
# print(model_tree,v->summary(v.val),"\t")


# T_base = CoordinateTransformations.Translation(0.0,0.0,0.0) ∘ CoordinateTransformations.LinearMap(LDrawParser.LDRAW_BASE_FRAME)
# SCALE = 0.01
# LDrawParser.change_coordinate_system!(model,T_base,SCALE)

# m = model.models["20009 - AT-TE Walker.mpd"]
# id_generator = DuplicateIDGenerator{String}()
# for step in m.steps
#     for ref in step.lines
#         global vis_root
#         @show ref
#         if !LDrawParser.has_part(model,ref.file)
#             @warn "$ref not found in part"
#             continue
#         end
#         p = model.parts[ref.file]
#         name = id_generator(p.name)
#         vec = LDrawParser.extract_surface_geometry(p)
#         M = GeometryBasics.Mesh(coordinates(vec),faces(vec))
#         setobject!(vis_root[name], M)
#         tr = LDrawParser.build_transform(ref)
#         settransform!(vis_root[name], tr)
#     end
# end
# delete!(vis)


# # VISUALIZE ROBOT PLACEMENT AROUND PARTS
#
# part_keys = sort(collect(keys(model.parts))[1:10])
# parts = Dict(k=>model.parts[k] for k in part_keys)
# transport_model = (
#     robot_radius = 15.0,
#     max_area_per_robot = 10000.0, #3000.0,
#     max_volume_per_robot = 1000000.0 #20000.0,
# )
# for (k,part) in parts
#     points = map(T, LDrawParser.extract_points(part))
#     if isempty(points)
#         println("Part $k has no geometry!")
#         continue
#     end
#     geom=map(SVector,points)
#     try
#         support_pts = HierarchicalGeometry.select_support_locations(
#             geom,transport_model)
#         polygon = VPolygon(convex_hull(map(p->p[1:2],geom)))
#
#         plt = plot(polygon,aspectratio=1,alpha=0.4)
#         scatter!(plt,map(p->p[1],geom),map(p->p[2],geom),label=k) #,legend=false)
#         plot!(plt,map(p->Ball2(p,transport_model.robot_radius),support_pts), aspectratio=1, alpha=0.4)
#         display(plt)
#     catch e
#         bt = catch_backtrace()
#         showerror(stdout,e,bt)
#     end
# end

# # construct model graph
# model_graph = construct_assembly_graph(model)
# model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
# # @assert maximum(map(v->indegree(model_tree,v),vertices(model_tree))) == 1
# print(model_tree,v->summary(v.val),"\t")

# sched = LDrawParser.construct_model_schedule(model)
# model_spec = LDrawParser.extract_single_model(sched,"20009 - AT-TE Walker.mpd")

# GraphUtils.validate_graph(model_spec)
