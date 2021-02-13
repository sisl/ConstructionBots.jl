using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using TaskGraphs
using JuMP
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)

using LightGraphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations
using StaticArrays
using LinearAlgebra

using PyCall
rvo = pyimport("rvo2")
ConstructionBots.set_rvo_python_module!(rvo)
ConstructionBots.set_rvo_default_max_speed!(3.0)

using MeshCat
const MESHCAT_GRID_DIMS = ((-10.0,10.0),(-10.0,10.0))
using Plots
using Random
Random.seed!(0);

using Logging
global_logger(ConsoleLogger(stderr, Logging.Info))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))

# Start MeshCat viewer
vis = Visualizer()
render(vis)

reset_all_id_counters!()
reset_all_invalid_id_counters!()

# factor by which to scale LDraw model (because MeshCat bounds are hard to adjust)
# NUM_ROBOTS          = 20
NUM_ROBOTS          = 2
MODEL_SCALE         = 0.01
ROBOT_HEIGHT        = 10*MODEL_SCALE
ROBOT_RADIUS        = 25*MODEL_SCALE
set_default_robot_geom!(
    Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,ROBOT_HEIGHT), ROBOT_RADIUS)
)

## LOAD LDRAW FILE
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","Millennium Falcon.mpd")
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","ATTEWalker.mpd")
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","stack1.ldr")
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","big_stack.ldr")
filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","four_stack.mpd")
model = parse_ldraw_file(filename)
populate_part_geometry!(model)
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),MODEL_SCALE)

## CONSTRUCT MODEL SPEC
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec)
# model_spec = ConstructionBots.extract_single_model(spec,"20009 - AT-TE Walker.mpd")
@assert GraphUtils.validate_graph(model_spec)
display_graph(model_spec,scale=1,enforce_visited=true)

## CONSTRUCT SceneTree
id_map = ConstructionBots.build_id_map(model,model_spec)
assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
print(scene_tree,v->"$(summary(node_id(v))) : $(get(id_map,node_id(v),nothing))","\t")
# Define TransportUnit configurations
ConstructionBots.init_transport_units!(scene_tree;robot_radius = 0.5)
# validate SceneTree
root = get_node(scene_tree,collect(get_all_root_nodes(scene_tree))[1])
validate_tree(HierarchicalGeometry.get_transform_node(root))
validate_embedded_tree(scene_tree,v->HierarchicalGeometry.get_transform_node(get_node(scene_tree,v)))
# visualize
display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Add some robots to scene tree
vtxs = ConstructionBots.construct_vtx_array(;spacing=(1.0,1.0,0.0), ranges=(-10:10,-10:10,0:0))
robot_vtxs = draw_random_uniform(vtxs,NUM_ROBOTS)
ConstructionBots.add_robots_to_scene!(scene_tree,robot_vtxs,[default_robot_geom()])
display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Compute overapproximated geometry
# Add temporary dummy robots ############################
ConstructionBots.add_temporary_invalid_robots!(scene_tree;with_edges=true)
display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))
HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
# remove_geometry!(scene_tree,HypersphereKey())
@assert all(map(node->has_vertex(node.geom_hierarchy,HypersphereKey()), get_nodes(scene_tree)))
HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
@assert all(map(node->has_vertex(node.geom_hierarchy,HyperrectangleKey()), get_nodes(scene_tree)))
# Remove temporary dummy robots ############################
ConstructionBots.remove_temporary_invalid_robots!(scene_tree)
display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Construct Partial Schedule
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
sched = construct_partial_construction_schedule(model,model_spec,scene_tree,id_map)
# Check if schedule graph and embedded transform tree are valid
@assert validate_schedule_transform_tree(sched)
# sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,500)
# display_graph(sched2,scale=1,enforce_visited=true)
# display_graph(sched,scale=1,enforce_visited=true)

## Generate staging plan
staging_circles = ConstructionBots.generate_staging_plan!(scene_tree,sched)

# Move objects away from the staging plan
MAX_CARGO_HEIGHT = maximum(map(n->get_base_geom(n,HyperrectangleKey()).radius[3]*2,
    filter(n->matches_template(TransportUnitNode,n),get_nodes(scene_tree))))
vtxs = ConstructionBots.construct_vtx_array(;
    origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
    obstacles=collect(values(staging_circles)))
NUM_OBJECTS = length(filter(n->matches_template(ObjectNode,n),get_nodes(scene_tree)))
object_vtxs = draw_random_uniform(vtxs,NUM_OBJECTS)
ConstructionBots.select_initial_object_grid_locations!(sched,object_vtxs)

# Move assemblies up so they float above the robots
for node in get_nodes(scene_tree)
    if matches_template(AssemblyNode,node) 
        start_node = get_node(sched,AssemblyComplete(node))
        tform = CT.Translation(0.0,0.0,MAX_CARGO_HEIGHT) ∘ local_transform(start_config(start_node))
        set_local_transform!(start_config(start_node),tform)
    end
end

# Make sure all transforms line up
ConstructionBots.calibrate_transport_tasks!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

# Task Assignments
ConstructionBots.add_dummy_robot_go_nodes!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

tg_sched = ConstructionBots.convert_to_operating_schedule(sched)
# Greedy Assignment with enforced build-step ordering
# milp_model = SparseAdjacencyMILP()
milp_model = ConstructionBots.GreedyOrderedAssignment()
milp_model = formulate_milp(milp_model,tg_sched,scene_tree)
optimize!(milp_model)
validate_schedule_transform_tree(ConstructionBots.convert_from_operating_schedule(typeof(sched),tg_sched)
    ;post_staging=true)
update_project_schedule!(nothing,milp_model,tg_sched,scene_tree)
@assert validate(tg_sched)
display_graph(tg_sched,scale=3,enforce_visited=true) |> PDF("/home/kylebrown/Desktop/sched.pdf")

# sched2 = ConstructionBots.extract_small_sched_for_plotting(tg_sched,200)
# display_graph(sched2,scale=3,enforce_visited=true,aspect_stretch=(0.7,0.7))

## Visualize assembly
color_map = construct_color_map(model_spec,id_map)
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
setvisible!(sphere_nodes,true)
setvisible!(rect_nodes,true)
setvisible!(staging_nodes,true)
setvisible!(sphere_nodes,false)
setvisible!(rect_nodes,false)
setvisible!(staging_nodes,false)

# restore correct configuration
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
update_visualizer!(scene_tree,vis_nodes)
# set staging plan and visualize
set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
update_visualizer!(scene_tree,vis_nodes)
# Visualize construction
# visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;dt=0.1)

# RVO
ConstructionBots.set_default_loading_speed!(0.5)
ConstructionBots.set_rvo_default_time_step!(1/40.0)
ConstructionBots.set_rvo_default_neighbor_distance!(4.0)
ConstructionBots.set_rvo_default_min_neighbor_distance!(3.0)
ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(;horizon=2.0))
env = PlannerEnv(sched=tg_sched,scene_tree=scene_tree)
active_nodes = (get_node(tg_sched,v) for v in env.cache.active_set)
ConstructionBots.rvo_add_agents!(scene_tree,active_nodes)

update_visualizer_function(env) = begin
    update_visualizer!(env.scene_tree,vis_nodes)
    render(vis)
end

ConstructionBots.simulate!(env,update_visualizer_function)

# VISUALIZE ROBOT PLACEMENT AROUND PARTS

# part_keys = sort(collect(keys(model.parts))[1:10])
# parts = Dict(k=>model.parts[k] for k in part_keys)
# transport_model = (
#     robot_radius = 0.5,
#     max_area_per_robot = 100.0, #3000.0,
#     max_volume_per_robot = 100.0 #20000.0,
# )
# for (k,part) in parts
#     points = map(SVector{3,Float64}, LDrawParser.extract_points(part))
#     if isempty(points)
#         println("Part $k has no geometry!")
#         continue
#     end
#     geom=map(SVector,points)
#     try
#         support_pts = HierarchicalGeometry.select_support_locations(
#             geom,transport_model)
#         polygon = VPolygon(convex_hull(map(p->p[1:2],geom)))

#         plt = plot(polygon,aspectratio=1,alpha=0.4)
#         scatter!(plt,map(p->p[1],geom),map(p->p[2],geom),label=k) #,legend=false)
#         plot!(plt,map(p->Ball2(p,transport_model.robot_radius),support_pts), aspectratio=1, alpha=0.4)
#         display(plt)
#     catch e
#         bt = catch_backtrace()
#         showerror(stdout,e,bt)
#     end
# end
