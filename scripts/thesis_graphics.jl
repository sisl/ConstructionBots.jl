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

using Logging
global_logger(ConsoleLogger(stderr, Logging.Info))
# global_logger(ConsoleLogger(stderr, Logging.Debug))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))

# Start MeshCat viewer
vis = Visualizer()
render(vis)

reset_all_id_counters!()
reset_all_invalid_id_counters!()
Random.seed!(0);

# factor by which to scale LDraw model (because MeshCat bounds are hard to adjust)
# MODEL_SCALE         = 0.01
MODEL_SCALE         = 0.0075
NUM_ROBOTS          = 30

## LOAD LDRAW FILE
# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","Millennium Falcon.mpd")
# NUM_ROBOTS          = 200
# MODEL_SCALE         = 0.005
# model_name = "simple_quad_stack.mpd"
# model_name = "DemoStack.mpd"
# model_name = "ATTEWalker.mpd"
# model_name = "stack1.ldr"
# model_name = "big_stack.ldr"
# model_name = "triple_stack.mpd"
model_name = "quad_nested.mpd"
# model_name = "small_quad_nested.mpd"
filename = joinpath(dirname(pathof(LDrawParser)),"..","assets",model_name)
# NUM_ROBOTS          = 40
# MODEL_SCALE         = 0.01

base_graphics_path = "/scratch/Repositories/Sandbox/thesis_graphics/LEGO"
graphics_path = joinpath(base_graphics_path,model_name)
mkpath(graphics_path)

ROBOT_HEIGHT        = 10*MODEL_SCALE
ROBOT_RADIUS        = 25*MODEL_SCALE
set_default_robot_geom!(
    Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,ROBOT_HEIGHT), ROBOT_RADIUS)
)

model = parse_ldraw_file(filename)
populate_part_geometry!(model);
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),MODEL_SCALE);

## CONSTRUCT MODEL SPEC
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec)
id_map = ConstructionBots.build_id_map(model,model_spec)
color_map = construct_color_map(model_spec,id_map)
@assert GraphUtils.validate_graph(model_spec)
plt = display_graph(model_spec,scale=1) #,enforce_visited=true)
display(plt)
draw(PDF(joinpath(graphics_path,"model_spec.pdf")),plt)


## CONSTRUCT SceneTree
assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
print(scene_tree,v->"$(summary(node_id(v))) : $(get(id_map,node_id(v),nothing))","\t")
# Define TransportUnit configurations
HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
ConstructionBots.init_transport_units!(scene_tree;robot_radius = 2*ROBOT_RADIUS)
# validate SceneTree
root = get_node(scene_tree,collect(get_all_root_nodes(scene_tree))[1])
validate_tree(HierarchicalGeometry.get_transform_node(root))
validate_embedded_tree(scene_tree,v->HierarchicalGeometry.get_transform_node(get_node(scene_tree,v)))
# visualize
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Add some robots to scene tree
vtxs = ConstructionBots.construct_vtx_array(;spacing=(1.0,1.0,0.0), ranges=(-10:10,-10:10,0:0))
robot_vtxs = draw_random_uniform(vtxs,NUM_ROBOTS)
ConstructionBots.add_robots_to_scene!(scene_tree,robot_vtxs,[default_robot_geom()])
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

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
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Construct Partial Schedule
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
sched = construct_partial_construction_schedule(model,model_spec,scene_tree,id_map)
# Check if schedule graph and embedded transform tree are valid
@assert validate_schedule_transform_tree(sched)
# sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,500)
# display_graph(sched2,scale=1,enforce_visited=true)
# display_graph(sched,scale=1) #,enforce_visited=true)

## Generate staging plan
staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree,sched;
    buffer_radius = 5*default_robot_radius(),
    build_step_buffer_radius=default_robot_radius()/2,
)
# plot staging plan
plt = plot_staging_plan_2d(sched,scene_tree,
    _fontsize=20pt,
    nominal_width=20cm,
    _show_bounding_circs=true,
    _show_dropoffs=true,
    base_geom_layer=plot_assemblies(sched,scene_tree,fill_color=RGBA(0.0,0.0,0.0,0.0),geom_key=HyperrectangleKey())
    )
display(plt)
draw(PDF(joinpath(graphics_path,"staging_plan.pdf")),plt)


# Move objects away from the staging plan
MAX_CARGO_HEIGHT = maximum(map(n->get_base_geom(n,HyperrectangleKey()).radius[3]*2,
    filter(n->matches_template(TransportUnitNode,n),get_nodes(scene_tree))))
vtxs = ConstructionBots.construct_vtx_array(;
    origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
    obstacles=collect(values(staging_circles)),
    ranges=(-10:10,-10:10,0:2),
    )
NUM_OBJECTS = length(filter(n->matches_template(ObjectNode,n),get_nodes(scene_tree)))
object_vtxs = draw_random_uniform(vtxs,NUM_OBJECTS)
ConstructionBots.select_initial_object_grid_locations!(sched,object_vtxs)

# Move assemblies up so they float above the robots
for node in get_nodes(scene_tree)
    if matches_template(AssemblyNode,node) 
        start_node = get_node(sched,AssemblyComplete(node))
        current = global_transform(start_config(start_node))
        tform_error = CT.Translation(0.0,0.0,MAX_CARGO_HEIGHT-current.translation[3]) # difference to be made up in global frame
        rot_mat = CT.LinearMap(global_transform(get_parent(HG.get_transform_node(node))).linear)
        tform = local_transform(start_config(start_node)) ∘ inv(rot_mat) ∘ tform_error
        # tform = CT.Translation(0.0,0.0,MAX_CARGO_HEIGHT) ∘ local_transform(start_config(start_node))
        set_local_transform!(start_config(start_node),tform)
    end
end

# Make sure all transforms line up
ConstructionBots.calibrate_transport_tasks!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

# Task Assignments
ConstructionBots.add_dummy_robot_go_nodes!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

# Convert to OperatingSchedule
ConstructionBots.set_default_loading_speed!(10*default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(10*default_robot_radius())
tg_sched = ConstructionBots.convert_to_operating_schedule(sched)
## Black box MILP solver
TaskGraphs.set_default_optimizer_attributes!(
    "TimeLimit"=>50,
    MOI.Silent()=>false
    )
milp_model = SparseAdjacencyMILP()
## Greedy Assignment with enforced build-step ordering
# milp_model = ConstructionBots.GreedyOrderedAssignment(
#     greedy_cost = TaskGraphs.GreedyFinalTimeCost(),
# )
milp_model = formulate_milp(milp_model,tg_sched,scene_tree)
optimize!(milp_model)
validate_schedule_transform_tree(ConstructionBots.convert_from_operating_schedule(typeof(sched),tg_sched)
    ;post_staging=true)
update_project_schedule!(nothing,milp_model,tg_sched,scene_tree)
@assert validate(tg_sched)
# display_graph(tg_sched,scale=3,enforce_visited=true) |> PDF("/home/kylebrown/Desktop/sched.pdf")
# sched2 = ConstructionBots.extract_small_sched_for_plotting(tg_sched,200)
# display_graph(sched2,scale=3,enforce_visited=true,aspect_stretch=(0.9,0.9))

# Try assigning robots to "home" locations so they don't sit around in each others' way
go_nodes = [n for n in get_nodes(tg_sched) if matches_template(RobotGo,n) && is_terminal_node(tg_sched,n)]
home_vtxs = draw_random_uniform(vtxs,length(go_nodes))
for (vtx,n) in zip(home_vtxs,go_nodes)
    HG.set_desired_global_transform!(goal_config(n),
        CT.Translation(vtx[1],vtx[2],0.0) ∘ identity_linear_map()
    )
end

## Visualize staging plans to debug the rotational shuffling 
# vis_triads, vis_arrows, bounding_vis = visualize_staging_plan(vis,sched,scene_tree)
# delete!(vis_arrows)
# delete!(vis_triads)
# delete!(bounding_vis)

## Visualize assembly
delete!(vis)
vis_nodes, base_geom_nodes = populate_visualizer!(scene_tree,vis;
    color_map=color_map,
    # wireframe=true,
    material_type=MeshLambertMaterial)
sphere_nodes = show_geometry_layer!(scene_tree,vis_nodes,HypersphereKey())
rect_nodes = show_geometry_layer!(scene_tree,vis_nodes,HyperrectangleKey();
    color=RGBA{Float32}(1, 0, 0, 0.3),
)
staging_nodes = render_staging_areas!(vis,scene_tree,sched,staging_circles;
    dz=0.01,color=RGBA(0.4,0.0,0.4,0.5))
setvisible!(base_geom_nodes,true)
setvisible!(sphere_nodes,true)
setvisible!(rect_nodes,true)
setvisible!(staging_nodes,true)
# setvisible!(base_geom_nodes,false)
setvisible!(sphere_nodes,false)
setvisible!(rect_nodes,false)
setvisible!(staging_nodes,false)
render(vis)

# restore correct configuration
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
update_visualizer!(scene_tree,vis_nodes)
# set staging plan and visualize
set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
update_visualizer!(scene_tree,vis_nodes)
# Visualize construction
# visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;dt=0.1)


# render video!
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
update_visualizer!(scene_tree,vis_nodes)
anim = AnimationWrapper(0)
animate_preprocessing_steps!(
        vis,
        vis_nodes,
        scene_tree,
        sched,
        rect_nodes,
        ;
        dt_animate=0.0,
        dt=0.0,
        anim=anim,
    )
setanimation!(vis,anim.anim)
render(vis)
# open(joinpath(graphics_path,"animate_preprocessing.html"),"w") do io
#     write(io,static_html(vis))
# end
set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
update_visualizer!(scene_tree,vis_nodes)

# rvo
ConstructionBots.set_rvo_default_time_step!(1/40.0)
ConstructionBots.set_rvo_default_neighbor_distance!(16*default_robot_radius()) # 4
ConstructionBots.set_rvo_default_min_neighbor_distance!(10*default_robot_radius()) # 3
ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(;horizon=2.0))
ConstructionBots.set_staging_buffer_radius!(default_robot_radius())
env = PlannerEnv(
        sched=tg_sched,
        scene_tree=scene_tree,
        staging_circles=staging_circles
        )
active_nodes = (get_node(tg_sched,v) for v in env.cache.active_set)
ConstructionBots.rvo_add_agents!(scene_tree,active_nodes)

update_visualizer_function = construct_visualizer_update_function(vis,vis_nodes,staging_nodes;
    # anim=nothing,
    anim=anim,
    )


# Turn off RVO to see if the project can be completed if we don't worry about collision
# set_use_rvo!(false)
# set_avoid_staging_areas!(false)
set_use_rvo!(true)
set_avoid_staging_areas!(true)

ConstructionBots.simulate!(env,update_visualizer_function,max_time_steps=2000)
setanimation!(vis,anim.anim)
# open(joinpath(graphics_path,"construction_simulation.html"),"w") do io
#     write(io,static_html(vis))
# end
render(vis)

# Test circle_avoidance_policy
circles = [
    LazySets.Ball2(SVector(0.0,0.0),1.0),
    # LazySets.Ball2(SVector(1.0,-3.0),1.0),
    LazySets.Ball2(SVector(0.0,4.0),2.0),
    ]
goal = [0.0,8.0]
pos =  [0.0,-6.0]
agent_radius = default_robot_radius()
dt = 0.025
vmax = 1.0
delete!(vis)
setobject!(vis[:robot],default_robot_geom(),MeshLambertMaterial(color=RGB(0.1,0.1,0.1)))
settransform!(vis[:robot],CT.Translation(pos...,0.0))
setobject!(vis[:goal],default_robot_geom(),MeshLambertMaterial(color=RGBA(0.0,1.0,0.0,0.2)))
settransform!(vis[:goal],CT.Translation(goal...,0.0))
for (i,c) in enumerate(circles)
    setobject!(vis[:circles][string(i)],convert(GeometryBasics.Sphere,HG.project_to_3d(c)),
     MeshLambertMaterial(color=RGBA(1.0,0.0,0.0,0.1)))
end
for t in 1:1000
    goal_pt = ConstructionBots.circle_avoidance_policy(circles,agent_radius,pos,goal;buffer=0.5)
    vel = normalize(goal_pt - pos) * min(vmax,norm(goal_pt - pos)/dt)
    if any(isnan,vel)
        vel = [0.0,0.0]
        break
    end
    pos = pos .+ vel*dt
    # update visualizer
    settransform!(vis[:robot],CT.Translation(pos...,0.0))
    sleep(dt)
end



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
