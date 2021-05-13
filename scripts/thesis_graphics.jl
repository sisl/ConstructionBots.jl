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

using TOML
using Logging
global_logger(ConsoleLogger(stderr, Logging.Info))
# global_logger(ConsoleLogger(stderr, Logging.Warn))
# global_logger(ConsoleLogger(stderr, Logging.Debug))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))
Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers","render_tools.jl"))

# Start MeshCat viewer
vis = Visualizer()
MeshCat.render(vis)

reset_all_id_counters!()
reset_all_invalid_id_counters!()
Random.seed!(0);

# factor by which to scale LDraw model (because MeshCat bounds are hard to adjust)
# MODEL_SCALE         = 0.01

## LOAD LDRAW FILE

# project_name = "quad_nested.mpd"
# MODEL_SCALE         = 0.003
# NUM_ROBOTS          = 50
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)

# project_name = "ATTEWalker.mpd"
# MODEL_SCALE         = 0.003
# NUM_ROBOTS          = 36
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)

# project_name = "StarDestroyer.mpd"
# MODEL_SCALE         = 0.004
# NUM_ROBOTS          = 100
# ROBOT_SCALE         = MODEL_SCALE * 0.7
# MAX_STEPS           = 20000
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 0.5

# project_name = "Saturn.mpd"
# MODEL_SCALE         = 0.001
# NUM_ROBOTS          = 100
# ROBOT_SCALE         = MODEL_SCALE*4
# MAX_STEPS           = 20000
# OBJECT_VTX_RANGE =(-36:36,-36:36,0:8)
# HOME_VTX_RANGE    = (-34:34,-34:34, 0:0)
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 0.5

# project_name = "MillenniumFalcon.mpd"
# MODEL_SCALE         = 0.001
# NUM_ROBOTS          = 200
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-26:26,-26:26, 0:10)

# project_name = "X-wingFighter.mpd"
# MODEL_SCALE         = 0.0035
# NUM_ROBOTS          = 100
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-14:0.5:14,-14:0.5:14, 0:0)
# HOME_VTX_RANGE    = (-22:22,-22:22, 0:0)
# MAX_STEPS           = 8000
# STAGING_BUFFER_FACTOR = 2.2
# BUILD_STEP_BUFFER_FACTOR = 0.5

project_name = "X-wingMini.mpd"
MODEL_SCALE         = 0.007
ROBOT_SCALE         = MODEL_SCALE * 0.7
NUM_ROBOTS          = 30
OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:0)
HOME_VTX_RANGE      = (-10:10,-10:10, 0:0)
MAX_STEPS           = 8000
STAGING_BUFFER_FACTOR = 1.5
BUILD_STEP_BUFFER_FACTOR = 1.5

# project_name = "tractor.mpd"
# MODEL_SCALE         = 0.008
# ROBOT_SCALE         = MODEL_SCALE * 0.7
# NUM_ROBOTS          = 12
# MAX_STEPS           = 4000
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE      = (-10:10, -10:10, 0:1)
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 1.5

# project_name = "colored_8x8.ldr"
# MODEL_SCALE         = 0.01
# ROBOT_SCALE         = MODEL_SCALE * 0.9
# NUM_ROBOTS          = 25
# MAX_STEPS           = 2500
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:0)
# HOME_VTX_RANGE      = (-10:10,-10:10, 0:0)
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 1.5

# project_name = "small_quad_nested.mpd"
# NUM_ROBOTS          = 40
# MODEL_SCALE         = 0.01
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)

filename = joinpath(dirname(pathof(LDrawParser)),"..","assets",project_name)

base_graphics_path = "/scratch/Repositories/Sandbox/thesis_graphics/LEGO"
graphics_path = joinpath(base_graphics_path,project_name)
mkpath(graphics_path)

ROBOT_HEIGHT        = 10*ROBOT_SCALE
ROBOT_RADIUS        = 25*ROBOT_SCALE
set_default_robot_geom!(
    Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,ROBOT_HEIGHT), ROBOT_RADIUS)
)
FR.set_render_param!(:Radius,:Robot,ROBOT_RADIUS)

model = parse_ldraw_file(filename)
populate_part_geometry!(model);
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),MODEL_SCALE);

## CONSTRUCT MODEL SPEC
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec)
id_map = ConstructionBots.build_id_map(model,model_spec)
color_map = construct_color_map(model_spec,id_map)
@assert GraphUtils.validate_graph(model_spec)
# plt = display_graph(model_spec,scale=1,aspect_stretch=(1.2,1.0)) #,enforce_visited=true)
# display(plt)
# draw(PDF(joinpath(graphics_path,"model_spec.pdf")),plt)

# Render the model spec with pictures of the assembly stages
let
    # labels = Dict()
    # for n in get_nodes(model_spec)
    #     if matches_template(BuildingStep,n)
    #         for np in node_iterator(model_spec, outneighbors(model_spec,n))
    #             if matches_template(SubModelPlan,np)
    #                 labels[node_id(n)] = string("A",get_id(id_map[node_val(n).parent]))
    #                 labels[id_map[node_val(n).parent]] = labels[node_id(n)]
    #                 break
    #             end
    #         end
    #     end
    # end
    # plt = render_model_spec_with_pictures(model_spec,
    #     base_image_path=joinpath(dirname(pathof(LDrawParser)),"..","assets","tractor"),
    #     scale=4,
    #     aspect_stretch=(0.8,0.4),
    #     picture_scale=1.8,
    #     labels = labels,
    #     label_pos = (0.1,0.1),
    #     label_fontsize = 18pt,
    #     label_radius = 0.6cm,
    #     label_bg_color = RGB(0.9,0.9,0.9),
    #     label_stroke_color = "black",
    #     )
    # # convert to pdf later
    # # draw(SVG(joinpath(graphics_path,"model_spec_with_pictures.svg")),plt)
end

## CONSTRUCT SceneTree
assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
print(scene_tree,v->"$(summary(node_id(v))) : $(get(id_map,node_id(v),nothing))","\t")
# Define TransportUnit configurations
HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
_, cvx_hulls = ConstructionBots.init_transport_units!(scene_tree;
    robot_radius = ROBOT_RADIUS,
    )

# ConstructionBots.init_transport_units!(scene_tree)
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
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))
HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
# remove_geometry!(scene_tree,HypersphereKey())
@assert all(map(node->has_vertex(node.geom_hierarchy,HypersphereKey()), get_nodes(scene_tree)))
HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
@assert all(map(node->has_vertex(node.geom_hierarchy,HyperrectangleKey()), get_nodes(scene_tree)))

# render transport units 2D
let
    # base_rect2d = get_base_geom(get_node(scene_tree,TransportUnitNode(AssemblyID(1))),HyperrectangleKey())
    # for n in get_nodes(scene_tree)
    #     if matches_template(TransportUnitNode,n)
    #         cargo = get_node(scene_tree,cargo_id(n))
    #         plt = render_transport_unit_2d(scene_tree,n,cvx_hulls[node_id(cargo)],
    #             hull_thickness=0.015,
    #             line_thickness=0.0025,
    #             rect2d = Hyperrectangle(get_base_geom(n,HyperrectangleKey()).center,base_rect2d.radius),
    #             xpad = (0.2,0.2),
    #             ypad = (0.2,0.2),
    #             )
    #         outpath = joinpath(graphics_path,"transport_configs")
    #         mkpath(outpath)
    #         filename = joinpath(outpath,"$(typeof(cargo))_$(get_id(node_id(cargo))).pdf")
    #         draw(PDF(filename),plt)
    #     end
    # end
end

# Remove temporary dummy robots ############################
ConstructionBots.remove_temporary_invalid_robots!(scene_tree)
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))

## Construct Partial Schedule
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
sched = construct_partial_construction_schedule(model,model_spec,scene_tree,id_map)
# Check if schedule graph and embedded transform tree are valid
@assert validate_schedule_transform_tree(sched)
# display_graph(scene_tree,grow_mode=:from_top,align_mode=:root_aligned,aspect_stretch=(0.7,6.0))


let
    # sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,25;
    #     frontier=[get_vtx(sched,ProjectComplete(1))],
    #     )
    # plt = display_graph(sched2,scale=1,grow_mode=:from_left)
    # display(plt)
    # draw(PDF(joinpath(graphics_path,"sub_schedule_top.pdf")),plt)
    # for a in [2,4,6]
    #     sched3 = ConstructionBots.extract_small_sched_for_plotting(sched,55;
    #         frontier=[get_vtx(sched,AssemblyComplete(get_node(scene_tree,AssemblyID(a)))),
    #             [get_vtx(sched,outneighbors(sched,get_node(sched,RobotStart(
    #                 get_node(scene_tree,RobotID(i)))))[1]) for i in [1]]...,
    #         ],
    #         )
    #     let
    #         _node_type_check(n) = matches_template((ObjectStart,AssemblyStart,AssemblyComplete,FormTransportUnit,TransportUnitGo,DepositCargo,LiftIntoPlace),n)
    #         plt = display_graph(sched3,scale=1,grow_mode=:from_left,
    #             align_mode=:root_aligned,
    #             draw_node_function=(G,v)->draw_node(get_node(G,v);
    #                 title_text= _node_type_check(get_node(G,v)
    #                     ) ? string(GraphPlottingBFS._title_string(get_node(G,v)),"$(get_id(node_id(get_node(G,v))))") : GraphPlottingBFS._title_string(get_node(G,v)),
    #                 subtitle_text="",
    #                 title_scale = _node_type_check(get_node(G,v)
    #                     ) ? GraphPlottingBFS._title_text_scale(get_node(G,v)) : 0.45,
    #             ),
    #             pad=(0.0,0.0),
    #         ) 
    #         display(plt)
    #         draw(PDF(joinpath(graphics_path,"sub_schedule_assembly$(a).pdf")),plt)
    #     end
    #     codes = unique([GraphPlottingBFS._subtitle_string(n) for n in get_nodes(sched3)])
    #     cmap = distinguishable_colors(length(codes),lchoices=[50],cchoices=[30])
    #     color_dict = Dict(k=>c for (k,c) in zip(codes,cmap))
    #     bg_map = Dict('o'=>"white",'r'=>RGB(0.6,0.6,0.8),'a'=>RGB(0.8,0.8,0.8))
    #     plt = display_graph(sched3,scale=1,grow_mode=:from_left,
    #         draw_node_function=(G,v)->draw_node(get_node(G,v);
    #             subtitle_text="",
    #             text_color="black",
    #             # bg_color= findfirst("o",GraphPlottingBFS._subtitle_string(get_node(G,v))) === nothing ? RGB(0.8,0.8,0.8) : "white",
    #             bg_color= bg_map[GraphPlottingBFS._subtitle_string(get_node(G,v))[1]],
    #             title_scale=0.45,
    #             node_color=color_dict[GraphPlottingBFS._subtitle_string(get_node(G,v))],
    #         )
    #     )
    #     display(plt)
    #     draw(PDF(joinpath(graphics_path,"sub_schedule_assembly$(a)_colored.pdf")),plt)
    # end
    # sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,500;
    #     frontier=[get_vtx(sched,ProjectComplete(1))],
    #     )
    # plt = display_graph(sched2,scale=1,grow_mode=:from_left,
    #     draw_node_function=(G,v)->GraphPlottingBFS.draw_node(get_node(G,v);
    #         subtitle_text="",title_scale=0.4),
    #         align_mode=:split_aligned,
    # ) 
    # display(plt)
    # draw(PDF(joinpath(graphics_path,"schedule_unassigned.pdf")),plt)

end

# render each node type to a separate file
let
    # render_node_types_and_table(sched,scene_tree;graphics_path=joinpath(graphics_path,"node_table"))
end

## Generate staging plan
MAX_OBJECT_TRANSPORT_UNIT_RADIUS = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)
STAGING_PLAN_TIME = time()
staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree,sched;
    # buffer_radius = 3*HG.default_robot_radius(),
    buffer_radius=STAGING_BUFFER_FACTOR*MAX_OBJECT_TRANSPORT_UNIT_RADIUS,
    build_step_buffer_radius=BUILD_STEP_BUFFER_FACTOR*HG.default_robot_radius(),
    # build_step_buffer_radius=0.0,
    # buffer_radius = 0.0
);
STAGING_PLAN_TIME = time() - STAGING_PLAN_TIME
# plot staging plan
let
    # FR.set_render_param!(:Color,:Fill,:StagingCircle,nothing)
    # FR.set_render_param!(:Color,:Fill,:BoundingCircle,nothing)
    # FR.set_render_param!(:Color,:Fill,:DropoffCircle,nothing)
    # # FR.set_render_param!(:Color,:Label,:StagingCircle,nothing)
    # FR.set_render_param!(:Color,:Label,:StagingCircle,"red")
    # FR.set_render_param!(:Color,:Label,:DropoffCircle,nothing)
    # plt = plot_staging_plan_2d(sched,scene_tree,
    #     nominal_width=30cm,
    #     _show_bounding_circs=true,
    #     _show_dropoffs=true,
    #     base_geom_layer=Compose.compose(
    #         context(),
    #         Compose.linewidth(0.02pt),
    #         plot_assemblies(sched,scene_tree,
    #             # fill_color=RGBA(0.3,0.3,0.3,0.9),
    #             # stroke_color=RGBA(0.0,0.0,0.0,1.0),
    #             fill_color=RGBA(0.5,0.5,0.5,0.9),
    #             stroke_color=nothing,
    #             # geom_key=HyperrectangleKey(),
    #         )),
    #     _fontsize=30pt,
    #     bg_color=nothing,
    #     text_placement_func=circ-> (circ.center[1:2] .- [0.0,0.35-circ.radius])
    #     )
    # # build more clear staging plan graphic
    # bbox = staging_plan_bbox(sched)
    # circs = transform_final_assembly_staging_circles(sched,scene_tree,staging_circles)
    # for k in [4,5,6,7,8]
    #     delete!(circs,AssemblyID(k))
    # end
    # # delete!(circs,AssemblyID(1))
    # forms = [Compose.circle(v.center...,v.radius) for v in values(circs)]
    # stage_plt = Compose.compose(context(),plt,
    #     (context(units=UnitBox(bbox.origin...,bbox.widths...)),
    #         forms...,
    #         Compose.fill(nothing),
    #         # Compose.stroke(RGBA(1.0,1.0,1.0,1.0)),
    #         Compose.stroke(RGBA(0.5,0.5,0.5,1.0)),
    #         ),
    #     (context(),Compose.rectangle(),
    #         Compose.fill(RGB(0.25,0.25,0.25)),
    #         )
    #     )
    # display(stage_plt)
    # # draw(PDF(joinpath(graphics_path,"staging_plan.pdf")),stage_plt)
    # # draw(PDF(joinpath(graphics_path,"staging_plan_with_buffer.pdf")),stage_plt)
    # # draw(SVG(joinpath(graphics_path,"staging_plan.svg")),stage_plt)
    
    # # for each build step?
    # a = get_node(scene_tree,AssemblyID(1))
    # start = get_node(sched,AssemblyComplete(a))
    # steps = [n for n in node_iterator(sched,topological_sort_by_dfs(sched)) if matches_template(CloseBuildStep,n) && node_id(node_val(n).assembly)==node_id(a)]
    # components = Dict()
    # for build_step_node in steps
    #     components = merge(components,node_val(build_step_node).components)
    #     # filt_func = n -> (matches_template(CloseBuildStep,n) && node_id(node_val(n).assembly)==node_id(a) || (matches_template(DepositCargo,n) && haskey(a.components,cargo_id(entity(n)))))
    #     filt_func = n -> ( n === build_step_node || (matches_template(DepositCargo,n) && haskey(node_val(build_step_node).components,cargo_id(entity(n)))))
    #     # node_list = (n for n in node_iterator(sched,topological_sort_by_dfs(sched)) if matches_template(CloseBuildStep,n) && node_id(node_val(n).assembly)==node_id(a))
    #     node_list = (n for n in node_iterator(sched,topological_sort_by_dfs(sched)) if filt_func(n))
    #     plt = plot_staging_plan_2d(sched,scene_tree,
    #         nominal_width=30cm,
    #         _show_bounding_circs=true,
    #         _show_dropoffs=true,
    #         _show_intermediate_stages=true,
    #         _stage_stroke_color="red",
    #         base_geom_layer=Compose.compose(
    #             context(), 
    #             Compose.linewidth(0.02pt),
    #             [plot_base_geom_2d(get_node(scene_tree,k),scene_tree,
    #                 # tform=global_transform(goal_config(start)) ∘ tform,
    #                 tform=global_transform(goal_config(get_node(sched,
    #                     DepositCargo(get_node(scene_tree,TransportUnitNode(k)))))),
    #                 fill_color=RGBA(0.5,0.5,0.5,0.9),stroke_color=nothing,
    #                 ) for (k,tform) in components]...),
    #             # plot_base_geom_2d(a,scene_tree,
    #             #     fill_color=RGBA(0.5,0.5,0.5,0.9),stroke_color=nothing,)),
    #         _fontsize=30pt,
    #         bg_color=nothing,
    #         text_placement_func=circ-> (circ.center[1:2] .- [0.0,0.35-circ.radius]),
    #         node_list=node_list,
    #         )
    #     display(plt)
    # end
end

if project_name == "X-wingFighter.mpd" || project_name == "X-wingMini.mpd"
    # ac = get_node(sched,first(inneighbors(sched,ProjectComplete(1))))
    # tform = global_transform(goal_config(ac))
    # set_desired_global_transform!(
    #     goal_config(ac),
    #     CT.Translation(-12.0,6.0,0.0) ∘ global_transform(goal_config(ac)),
    #     )
    ac = get_node(sched,first(inneighbors(sched,ProjectComplete(1))))
    circ_center = HG.get_center(get_cached_geom(node_val(ac).outer_staging_circle))
    @assert has_parent(goal_config(ac),goal_config(ac))
    tform = relative_transform(
        CT.Translation(circ_center...),
        global_transform(goal_config(ac)),
        )
    set_local_transform!(goal_config(ac),tform)
end

# construct highway
# Revise.includet("/home/kylebrown/.julia/dev/ConstructionBots/src/highway.jl")

# construct_nested_highway(sched,scene_tree,staging_circles)

# Move objects away from the staging plan
MAX_CARGO_HEIGHT = maximum(map(n->get_base_geom(n,HyperrectangleKey()).radius[3]*2,
    filter(n->matches_template(TransportUnitNode,n),get_nodes(scene_tree))))
vtxs = ConstructionBots.construct_vtx_array(;
    origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
    # obstacles=collect(values(staging_circles)),
    obstacles=[HG.project_to_2d(get_cached_geom(node_val(n).outer_staging_circle)) for n in get_nodes(sched) if matches_template(AssemblyComplete,n)],
    ranges=OBJECT_VTX_RANGE,
    )
NUM_OBJECTS = length(filter(n->matches_template(ObjectNode,n),get_nodes(scene_tree)))
object_vtxs = draw_random_uniform(vtxs,NUM_OBJECTS)
ConstructionBots.select_initial_object_grid_locations!(sched,object_vtxs)

# Move assemblies up so they float above the robots
# TODO Debug error here
for node in get_nodes(scene_tree)
    if matches_template(AssemblyNode,node) 
        start_node = get_node(sched,AssemblyComplete(node))
        # raise start 
        current = global_transform(start_config(start_node))
        rect = current(get_base_geom(node,HyperrectangleKey()))
        dh = MAX_CARGO_HEIGHT - (rect.center .- rect.radius)[3]
        # @show summary(node_id(node)), dh
        set_desired_global_transform_without_affecting_children!(
        # set_desired_global_transform!(
            start_config(start_node),
            CT.Translation(current.translation[1:2]...,dh) ∘ CT.LinearMap(current.linear)
        )

        # tform_error = CT.Translation(0.0,0.0,MAX_CARGO_HEIGHT-current.translation[3]) # difference to be made up in global frame
        # rot_mat = CT.LinearMap(global_transform(get_parent(HG.get_transform_node(node))).linear)
        # tform = local_transform(start_config(start_node)) ∘ inv(rot_mat) ∘ tform_error
        # set_local_transform!(start_config(start_node),tform)
    end
end
# Make sure all transforms line up
ConstructionBots.calibrate_transport_tasks!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)

# Task Assignments
ConstructionBots.add_dummy_robot_go_nodes!(sched)
@assert validate_schedule_transform_tree(sched;post_staging=true)
let 
    # # Plot partial schedule with dummy RobotGo nodes
    # for a in [1,2,4,6]
    #     frontier = [get_vtx(sched,AssemblyComplete(get_node(scene_tree,AssemblyID(a))))]
    #     sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,25;frontier=frontier)
    #     frontier = [get_vtx(sched,AssemblyComplete(get_node(scene_tree,AssemblyID(a))))]
    #     for n in get_nodes(sched)
    #         if matches_template(RobotGo,n)
    #             for np in get_nodes(sched2)
    #                 if has_edge(sched,n,np) || has_edge(sched,np,n)
    #                     push!(frontier,get_vtx(sched,n))
    #                 end
    #             end
    #         end
    #     end
    #     # r = get_node(sched,RobotStart(get_node(scene_tree,RobotID(1))))
    #     # push!(frontier,outneighbors(sched,r)[1])
    #     sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,52;frontier=frontier)
    #     _node_type_check(n) = matches_template((ObjectStart,AssemblyStart,AssemblyComplete,FormTransportUnit,TransportUnitGo,DepositCargo,LiftIntoPlace),n)
    #     plt = display_graph(sched2,scale=1,grow_mode=:from_left,
    #         draw_node_function=(G,v)->draw_node(get_node(G,v);
    #             title_text= _node_type_check(get_node(G,v)
    #                 ) ? string(GraphPlottingBFS._title_string(get_node(G,v)),"$(get_id(node_id(get_node(G,v))))") : GraphPlottingBFS._title_string(get_node(G,v)),
    #             subtitle_text="",
    #             title_scale = _node_type_check(get_node(G,v)
    #                 ) ? GraphPlottingBFS._title_text_scale(get_node(G,v)) : 0.45,
    #         ),
    #         # align_mode=:root_aligned,
    #         align_mode=:split_aligned,
    #         pad=(0.0,0.0),
    #     )
    #     display(plt)
    #     draw(PDF(joinpath(graphics_path,"sub_graph$(a)_dummy_go_nodes.pdf")),plt)
    # end
    # frontier = []
    # for i in 1:5
    #     r = get_node(sched,RobotStart(get_node(scene_tree,RobotID(i))))
    #     push!(frontier,outneighbors(sched,r)[1])
    # end
    # sched2 = ConstructionBots.extract_small_sched_for_plotting(sched,50;frontier=frontier)
    # plt = display_graph(sched2,scale=1,grow_mode=:from_left,
    #     draw_node_function=(G,v)->draw_node(get_node(G,v);
    #         subtitle_text="",
    #         title_scale=0.45,
    #     ),
    #     align_mode=:split_aligned,
    #     pad=(0.0,0.0),
    # )
    # draw(PDF(joinpath(graphics_path,"sub_graph_robots_alone.pdf")),plt)

end

# Convert to OperatingSchedule
ConstructionBots.set_default_loading_speed!(10*HG.default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(10*HG.default_robot_radius())
tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

## Black box MILP solver
# TaskGraphs.set_default_optimizer_attributes!(
#     "TimeLimit"=>75,
#     MOI.Silent()=>false
#     )
# milp_model = SparseAdjacencyMILP()
# if primal_status(milp_model) == MOI.NO_SOLUTION
#     milp_model = ConstructionBots.GreedyOrderedAssignment(
#         greedy_cost = TaskGraphs.GreedyFinalTimeCost(),
#     )
#     milp_model = formulate_milp(milp_model,tg_sched,scene_tree)
#     optimize!(milp_model)
# end
# ## Greedy Assignment with enforced build-step ordering
# D = TaskGraphs.construct_schedule_distance_matrix(tg_sched,scene_tree)
milp_model = ConstructionBots.GreedyOrderedAssignment(
    greedy_cost = TaskGraphs.GreedyFinalTimeCost(),
)
# milp_model = formulate_milp(milp_model,deepcopy(tg_sched),scene_tree)
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
home_vtx_candidates = ConstructionBots.construct_vtx_array(;
    origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
    # obstacles=collect(values(staging_circles)),
    # obstacles=[circle_obs],
    obstacles=[HG.project_to_2d(get_cached_geom(node_val(n).outer_staging_circle)) for n in get_nodes(sched) if matches_template(AssemblyComplete,n)],
    ranges=HOME_VTX_RANGE
    )
# home_vtxs = draw_random_uniform(vtxs,length(go_nodes))
home_vtxs = draw_random_uniform(home_vtx_candidates,length(go_nodes))
for (vtx,n) in zip(home_vtxs,go_nodes)
    HG.set_desired_global_transform!(goal_config(n),
        CT.Translation(vtx[1],vtx[2],0.0) ∘ identity_linear_map()
    )
end

# if project_name == "X-wingFighter.mpd"
#     ac = get_node(sched,first(inneighbors(sched,ProjectComplete(1))))
#     tform = global_transform(goal_config(ac))
#     set_desired_global_transform!(
#         goal_config(ac),
#         CT.Translation(-2.0,8.0,global_transform(goal_config(ac)).translation[end]),
#         )
# elseif project_name == "X-wingMini.mpd"
#     ac = get_node(sched,first(inneighbors(sched,ProjectComplete(1))))
#     circ_center = HG.get_center(get_cached_geom(node_val(ac).outer_staging_circle))
#     @assert has_parent(goal_config(ac),goal_config(ac))
#     tform = relative_transform(
#         CT.Translation(circ_center...),
#         global_transform(goal_config(ac)),
#         )
#     set_local_transform!(goal_config(ac),tform)
# end

## Visualize assembly
delete!(vis)
factory_vis = populate_visualizer!(scene_tree,vis;
    color_map=color_map,
    color=RGB(0.3,0.3,0.3),
    # wireframe=true,
    material_type=MeshLambertMaterial)
add_indicator_nodes!(factory_vis)
factory_vis.staging_nodes = render_staging_areas!(vis,scene_tree,sched,staging_circles;color=RGBA(0.4,0.0,0.4,0.5))
for (k,color) in [
        (HypersphereKey()=>RGBA(0.0,1.0,0.0,0.3)),
        (HyperrectangleKey()=>RGBA(1.0,0.0,0.0,0.3))
    ]
    show_geometry_layer!(factory_vis,k;color=color)
end
for (k,nodes) in factory_vis.geom_nodes 
    setvisible!(nodes,false)
end
setvisible!(factory_vis.geom_nodes[BaseGeomKey()],true)
setvisible!(factory_vis.active_flags,false)

MeshCat.render(vis)

let
    # HG.compute_approximate_geometries!(scene_tree,CylinderKey())
    # cylinder_nodes = show_geometry_layer!(scene_tree,vis_nodes,CylinderKey())
    # setvisible!(cylinder_nodes,false)
    # HG.compute_approximate_geometries!(scene_tree,OctagonalPrismKey())
    # prism_nodes = show_geometry_layer!(scene_tree,vis_nodes,OctagonalPrismKey())
    # setvisible!(prism_nodes,false)

    # # Visualize overapproximated geometry
    # a = get_node(scene_tree,AssemblyID(1))
    # geom = recurse_child_geometry(a,scene_tree,BaseGeomKey())
    # P = overapproximate(geom,HG.BufferedPolygonPrism(HG.regular_buffered_polygon(8,1.0;buffer=0.1)),0.01)
    # C = overapproximate(geom,Cylinder)
    # S = overapproximate(geom,Ball2{Float64,SVector{3,Float64}})
    # p_color = RGBA(1.0,0.7,0.0)
    # setobject!(vis[:prism_overapprox][:solid],GeometryBasics.Mesh(coordinates(P),faces(P)),MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:prism_overapprox][:wireframe],HG.get_wireframe_mesh(P),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=15pt))
    # setobject!(vis[:cylinder][:solid],C,MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:cylinder][:wireframe],HG.get_wireframe_mesh(C),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=15pt))
    # setobject!(vis[:sphere][:solid],convert(HyperSphere,S),MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:sphere][:wireframe],convert(HyperSphere,S),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=1pt))

    # # visualize overapproximated geometry of a transform unit
    # o = get_node(scene_tree,ObjectID(6))
    # tu = get_node(scene_tree,TransportUnitNode(o))
    # set_child!(scene_tree,tu,node_id(o))
    # setvisible!(vis_nodes,false)
    # setvisible!(vis_nodes[node_id(o)],true)
    # setvisible!(vis_nodes[node_id(tu)],true)
    # for (id,tform) in robot_team(tu)
    #     setvisible!(vis_nodes[id],true)
    #     set_child!(scene_tree,tu,id)
    # end
    # update_visualizer!(scene_tree,vis_nodes)
    # P = get_cached_geom(tu,OctagonalPrismKey())
    # C = get_cached_geom(tu,CylinderKey())
    # S = get_cached_geom(tu,HypersphereKey())
    # p_color = RGBA(1.0,0.7,0.0)
    # setobject!(vis[:prism_overapprox][:solid],GeometryBasics.Mesh(coordinates(P),faces(P)),MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:prism_overapprox][:wireframe],HG.get_wireframe_mesh(P),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=15pt))
    # setobject!(vis[:cylinder][:solid],C,MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:cylinder][:wireframe],HG.get_wireframe_mesh(C),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=15pt))
    # setobject!(vis[:sphere][:solid],convert(HyperSphere,S),MeshLambertMaterial(color=RGBA(RGB(p_color),0.05),depthWrite=false))
    # setobject!(vis[:sphere][:wireframe],convert(HyperSphere,S),MeshLambertMaterial(wireframe=true,color=p_color,wireframeLinewidth=1pt))
end

# restore correct configuration
HG.jump_to_final_configuration!(scene_tree;set_edges=true)
update_visualizer!(factory_vis)
# set staging plan and visualize
set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
update_visualizer!(factory_vis)
# Visualize construction
# visualize_construction_plan!(scene_tree,sched,vis,vis_nodes;dt=0.1)


# render video!
anim = AnimationWrapper(0)
# anim = nothing
atframe(anim,current_frame(anim)) do
    HG.jump_to_final_configuration!(scene_tree;set_edges=true)
    update_visualizer!(factory_vis)
    setvisible!(factory_vis.geom_nodes[HyperrectangleKey()],false)
    setvisible!(factory_vis.staging_nodes,false)
    setvisible!(factory_vis.active_flags,false)
end
step_animation!(anim)
animate_preprocessing_steps!(
        factory_vis,
        sched
        ;
        dt_animate=0.0,
        dt=0.0,
        anim=anim,
        interp_steps=40,
    )
atframe(anim,current_frame(anim)) do
    set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
    update_visualizer!(factory_vis)
end
setanimation!(vis,anim.anim)
# MeshCat.render(vis)
# open(joinpath(graphics_path,"animate_preprocessing.html"),"w") do io
#     write(io,static_html(vis))
# end
# set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
# update_visualizer!(scene_tree,vis_nodes)

# rvo
ConstructionBots.set_rvo_default_time_step!(1/40.0)
ConstructionBots.set_rvo_default_neighbor_distance!(16*HG.default_robot_radius()) # 4
ConstructionBots.set_rvo_default_min_neighbor_distance!(10*HG.default_robot_radius()) # 3
# ConstructionBots.set_rvo_default_neighbor_distance!(8*HG.default_robot_radius()) # 4
# ConstructionBots.set_rvo_default_min_neighbor_distance!(4*HG.default_robot_radius()) # 3
ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(;horizon=2.0))
ConstructionBots.set_staging_buffer_radius!(HG.default_robot_radius())
env = PlannerEnv(
        sched=tg_sched,
        scene_tree=scene_tree,
        staging_circles=staging_circles
        )
active_nodes = (get_node(tg_sched,v) for v in env.cache.active_set)
ConstructionBots.rvo_add_agents!(scene_tree,active_nodes)


# instantiate agent policies
# potential field
# agent <-> agent: cone + barrier
# cone(x1,x2,r1,r2,scale,height) = 

# agent = get_node(scene_tree,RobotID(1))
# policy = ConstructionBots.PotentialFieldController(
#     env = env,
#     agent=agent,
#     agent_radius = ROBOT_RADIUS,
#     vmax = ConstructionBots.get_rvo_max_speed(agent),
# )
static_potential_function = (x,r)->0.0
pairwise_potential_function = ConstructionBots.repulsion_potential
# pairwise_potential_function = (x,r,x2,r2)->ConstructionBots.repulsion_potential(x,r,x2,r2;
#     dr=2.5*HG.default_robot_radius())

for node in get_nodes(env.sched)
    if matches_template(Union{RobotStart,FormTransportUnit},node)
        n = entity(node)
# for n in get_nodes(scene_tree)
#     if matches_template(Union{RobotNode,TransportUnitNode},n)
        agent_radius = HG.get_radius(get_base_geom(n,HypersphereKey()))
        vmax = ConstructionBots.get_rvo_max_speed(n)
        env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
            nominal_policy = TangentBugPolicy(
                dt = env.dt,
                vmax = vmax,
                agent_radius = agent_radius,
            ),
            dispersion_policy = ConstructionBots.PotentialFieldController(
                env = env,
                agent = n,
                node = node,
                agent_radius = agent_radius,
                vmax = vmax,
                max_buffer_radius=2.5*HG.default_robot_radius(),
                static_potentials=static_potential_function,
                pairwise_potentials=pairwise_potential_function,
            )
        )
    end
end
# for n in get_nodes(scene_tree)
#     if matches_template(Union{RobotNode,TransportUnitNode},n)
#         policy = env.agent_policies[node_id(n)].dispersion_policy
#         @show summary(node_id(policy.node))
#         @show policy.dist_to_nearest_active_agent
#         ConstructionBots.update_dist_to_nearest_active_agent!(policy)
#         @show policy.dist_to_nearest_active_agent
#         @show policy.buffer_radius
#         ConstructionBots.update_buffer_radius!(policy)
#         @show policy.buffer_radius
#         # pos = HG.project_to_2d(global_transform(n).translation)
#         # ConstructionBots.compute_velocity_command!(policy,pos)
#     end
# end

update_visualizer_function = construct_visualizer_update_function(factory_vis;
    anim=anim,
    )

# Turn off RVO to see if the project can be completed if we don't worry about collision
# set_use_rvo!(false)
set_use_rvo!(true)

# record statistics
if use_rvo()
    prefix = "with_rvo"
else
    prefix = "without_rvo"
end
if isa(milp_model,AbstractGreedyAssignment)
    prefix = string("greedy_",prefix)
else
    prefix = string("optimal_",prefix)
end


status, TIME_STEPS = ConstructionBots.simulate!(env, update_visualizer_function,
    max_time_steps=MAX_STEPS,
    )
setanimation!(vis,anim.anim)
# mkpath(joinpath(graphics_path,prefix))
# open(joinpath(graphics_path,prefix,"construction_simulation.html"),"w") do io
#     write(io,static_html(vis))
# end
MeshCat.render(vis)

# setobject!(vis["agent_flag"],HyperSphere(Point(0.0,0.0,0.0),2*HG.default_robot_radius()),MeshLambertMaterial(color=RGBA(1.0,0.0,1.0,0.5)))
# active_nodes = Base.Iterators.cycle(map(v->get_node(env.sched,v),collect(env.cache.active_set)))
# i = 1
# node, i = iterate(active_nodes,i)
# @info "$(summary(node_id(node))) ---  $(summary(node_id(entity(node))))"
# settransform!(vis["agent_flag"],global_transform(entity(node)))

# animate camera path
rotate_camera!(vis,anim);
# rotate_camera!(vis,anim;radial_decay_factor=2e-6,θ_start=π/2,origin=[0.0,0.0,0.0]);
# setanimation!(vis,anim.anim)
# open(joinpath(graphics_path,prefix,"construction_simulation_rotating.html"),"w") do io
#     write(io,static_html(vis))
# end


# using Blink
# w = Blink.Window(async=false)
# loadfile(w,joinpath(graphics_path,"construction_simulation.html"))


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
