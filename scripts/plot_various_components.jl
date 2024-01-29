using ConstructionBots

using Printf
using Parameters
using Random
using StaticArrays
using LinearAlgebra
using Dates
using StatsBase
using JLD2
using ProgressMeter
using Logging
using TOML
using LazySets
using GeometryBasics
using CoordinateTransformations
using Rotations
using Graphs
using JuMP
using PyCall
using MeshCat
using SparseArrays
using LDrawParser
using Colors
using Measures
using Compose
using HiGHS
using ECOS



project_params = get_project_params(:tractor)

ldraw_file = project_params[:file_name]
project_name = project_params[:project_name]
model_scale = project_params[:model_scale]
num_robots = project_params[:num_robots]

block_save_anim = false
open_animation_at_end = true
save_animation_along_the_way = false
save_animation = false
anim_active_agents = false
anim_active_areas = false
update_anim_at_every_step = false

deconfliction_type = [:RVO, :TangentBugPolicy, :Dispersion]
tangent_bug_flag = true
dispersion_flag = true

assignment_mode = :greedy
milp_optimizer = :highs
optimizer_time_limit = 100
look_for_previous_milp_solution = false
save_milp_solution = false
previous_found_optimizer_time = 200

write_results = false
overwrite_results = false

robot_scale = model_scale * 0.7
robot_height = 10 * robot_scale
robot_radius = 25 * robot_scale
num_object_layers = 1
max_steps = 50
staging_buffer_factor = 1.2
build_step_buffer_factor = 0.5
base_results_path = joinpath(dirname(pathof(ConstructionBots)), "..", "results")
results_path = joinpath(base_results_path, project_name)
process_updates_interval = 25
save_anim_interval = 500
max_num_iters_no_progress = 10000
sim_batch_size = 50
log_level = Logging.Warn
milp_optimizer_attribute_dict = Dict()

ignore_rot_matrix_warning = true

rng = Random.MersenneTwister(1)

process_animation_tasks = save_animation || save_animation_along_the_way || open_animation_at_end

visualizer = nothing


filename = joinpath(dirname(pathof(ConstructionBots)), "..", "LDraw_files", ldraw_file)
@assert ispath(filename) "File $(filename) does not exist."

global_logger(ConsoleLogger(stderr, log_level))

# Adding additional attributes for GLPK, HiGHS, and Gurobi
time_limit_key = nothing

anim_prog_file_name = "visualization_"

anim_prog_path = joinpath(results_path, anim_prog_file_name)

sim_params = ConstructionBots.SimParameters(
    sim_batch_size,
    max_steps,
    process_animation_tasks,
    save_anim_interval,
    process_updates_interval,
    block_save_anim,
    update_anim_at_every_step,
    anim_active_agents,
    anim_active_areas,
    anim_prog_path,
    save_animation_along_the_way,
    max_num_iters_no_progress
)

ConstructionBots.reset_all_id_counters!()
ConstructionBots.reset_all_invalid_id_counters!()
ConstructionBots.set_default_robot_geom!(Cylinder(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, robot_height), robot_radius))
ConstructionBots.set_rvo_default_time_step!(1 / 40.0)

ConstructionBots.set_rvo_default_neighbor_distance!(16 * ConstructionBots.default_robot_radius())
ConstructionBots.set_rvo_default_min_neighbor_distance!(10 * ConstructionBots.default_robot_radius())

ConstructionBots.set_default_loading_speed!(50 * ConstructionBots.default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(50 * ConstructionBots.default_robot_radius())

ConstructionBots.set_staging_buffer_radius!(ConstructionBots.default_robot_radius()) # for tangent_bug policy

# Setting default optimizer for staging layout
ConstructionBots.set_default_geom_optimizer!(ECOS.Optimizer)
ConstructionBots.set_default_geom_optimizer_attributes!(MOI.Silent() => true)

model = parse_ldraw_file(filename; ignore_rotation_determinant=ignore_rot_matrix_warning)
populate_part_geometry!(model; ignore_rotation_determinant=ignore_rot_matrix_warning)
LDrawParser.change_coordinate_system!(model, ldraw_base_transform(), model_scale; ignore_rotation_determinant=ignore_rot_matrix_warning)

## CONSTRUCT MODEL SPEC
print("Constructing model spec...")
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec)
id_map = ConstructionBots.build_id_map(model, model_spec)
color_map = ConstructionBots.construct_color_map(model_spec, id_map)
@assert ConstructionBots.validate_graph(model_spec)
print("done!\n")

## CONSTRUCT SceneTree
print("Constructing scene tree...")
assembly_tree = ConstructionBots.construct_assembly_tree(model, model_spec, id_map)
scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
# @info print(scene_tree, v -> "$(summary(node_id(v))) : $(get(id_map,node_id(v),nothing))", "\t")
print("done!\n")

# Compute Approximate Geometry
print("Computing approximate geometry...")
start_geom_approx = time()
ConstructionBots.compute_approximate_geometries!(scene_tree, ConstructionBots.HypersphereKey())
ConstructionBots.compute_approximate_geometries!(scene_tree, ConstructionBots.HyperrectangleKey())
GEOM_APPROX_TIME = time() - start_geom_approx
print("done!\n")

# Define TransportUnit configurations
print("Configuring transport units...")
config_transport_units_time = time()
ConstructionBots.init_transport_units!(scene_tree; robot_radius=robot_radius)
config_transport_units_time = time() - config_transport_units_time
print("done!\n")

# validate SceneTree
print("Validating scene tree...")
root = ConstructionBots.get_node(scene_tree, collect(ConstructionBots.get_all_root_nodes(scene_tree))[1])
ConstructionBots.validate_tree(ConstructionBots.get_transform_node(root))
ConstructionBots.validate_embedded_tree(scene_tree, v -> ConstructionBots.get_transform_node(ConstructionBots.get_node(scene_tree, v)))
print("done!\n")

## Add robots to scene tree
robot_spacing = 5 * robot_radius
robot_start_box_side = ceil(sqrt(num_robots)) * robot_spacing
xy_range = (-robot_start_box_side/2:robot_spacing:robot_start_box_side/2)
vtxs = ConstructionBots.construct_vtx_array(; spacing=(1.0, 1.0, 0.0), ranges=(xy_range, xy_range, 0:0))

robot_vtxs = StatsBase.sample(rng, vtxs, num_robots; replace=false)

ConstructionBots.add_robots_to_scene!(scene_tree, robot_vtxs, [ConstructionBots.default_robot_geom()])

## Recompute approximate geometry for when the robot is transporting it
# Add temporary robots to the transport units and recalculate the bounding geometry
# then remove them after the new geometries are calcualted
ConstructionBots.add_temporary_invalid_robots!(scene_tree; with_edges=true)
ConstructionBots.compute_approximate_geometries!(scene_tree, ConstructionBots.HypersphereKey())
@assert all(map(node -> ConstructionBots.has_vertex(node.geom_hierarchy, ConstructionBots.HypersphereKey()), ConstructionBots.get_nodes(scene_tree)))
ConstructionBots.compute_approximate_geometries!(scene_tree, ConstructionBots.HyperrectangleKey())
@assert all(map(node -> ConstructionBots.has_vertex(node.geom_hierarchy, ConstructionBots.HyperrectangleKey()), ConstructionBots.get_nodes(scene_tree)))
ConstructionBots.remove_temporary_invalid_robots!(scene_tree)

## Construct Partial Schedule (without robots assigned)
print("Constructing partial schedule...")
ConstructionBots.jump_to_final_configuration!(scene_tree; set_edges=true)
sched = ConstructionBots.construct_partial_construction_schedule(model, model_spec, scene_tree, id_map)
@assert ConstructionBots.validate_schedule_transform_tree(sched)
print("done!\n")


#### Plot 2D hulls with robot carrying positions ####
ns = []
for n in scene_tree.nodes
    if ConstructionBots.matches_template(ConstructionBots.TransportUnitNode, n)
        plt = ConstructionBots.render_transport_unit_2d(scene_tree, n; scale=15cm)
        display(plt)
    end
end


#### Plot the schedule ####
_node_type_check(n) = ConstructionBots.matches_template((ObjectStart, AssemblyStart, AssemblyComplete, FormTransportUnit, TransportUnitGo, DepositCargo, LiftIntoPlace), n)

plt = ConstructionBots.display_graph(
    sched;
    grow_mode=:from_left,
    align_mode=:split_aligned,
    draw_node_function=(G, v) -> ConstructionBots.draw_node(ConstructionBots.get_node(G, v);
        title_text=_node_type_check(ConstructionBots.get_node(G, v)
        ) ? string(ConstructionBots._title_string(ConstructionBots.get_node(G, v)),
            "$(ConstructionBots.get_id(ConstructionBots.node_id(ConstructionBots.get_node(G,v))))") : ConstructionBots._title_string(ConstructionBots.get_node(G, v)),
        subtitle_text="",
        title_scale=_node_type_check(ConstructionBots.get_node(G, v)
        ) ? ConstructionBots._title_text_scale(ConstructionBots.get_node(G, v)) : 0.45,
    ),
    pad=(0.0, 0.0)
)
display(plt)


# Generate staging plan
max_object_transport_unit_radius = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)

staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree, sched;
    buffer_radius=staging_buffer_factor * max_object_transport_unit_radius,
    build_step_buffer_radius=build_step_buffer_factor * ConstructionBots.default_robot_radius()
)

#### Plot the staging area ####
ConstructionBots.plot_staging_area(
    sched, scene_tree, staging_circles;
    save_file_name="temp_staging_area.pdf",
    save_image=false
)

# Make sure all transforms line up
ConstructionBots.calibrate_transport_tasks!(sched)
@assert ConstructionBots.validate_schedule_transform_tree(sched; post_staging=true)

# Task Assignments
ConstructionBots.add_dummy_robot_go_nodes!(sched)
@assert ConstructionBots.validate_schedule_transform_tree(sched; post_staging=true)

ConstructionBots.set_default_loading_speed!(50 * ConstructionBots.default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(50 * ConstructionBots.default_robot_radius())

tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

milp_model = ConstructionBots.SparseAdjacencyMILP()
milp_model = ConstructionBots.GreedyOrderedAssignment(
    greedy_cost=ConstructionBots.GreedyFinalTimeCost(),
)
milp_model = ConstructionBots.formulate_milp(milp_model, tg_sched, scene_tree)

optimize!(milp_model)

validate_schedule_transform_tree(
    ConstructionBots.convert_from_operating_schedule(typeof(sched), tg_sched)
    ; post_staging=true
)
ConstructionBots.update_project_schedule!(nothing, milp_model, tg_sched, scene_tree)
@assert ConstructionBots.validate(tg_sched)

#### Plot the schedule with robots assigned ####
plt = ConstructionBots.display_graph(
    tg_sched;
    grow_mode=:from_left,
    align_mode=:split_aligned,
    draw_node_function=(G, v) -> ConstructionBots.draw_node(ConstructionBots.get_node(G, v);
        title_text=_node_type_check(ConstructionBots.get_node(G, v)
        ) ? string(ConstructionBots._title_string(ConstructionBots.get_node(G, v)),
            "$(ConstructionBots.get_id(ConstructionBots.node_id(ConstructionBots.get_node(G,v))))") : ConstructionBots._title_string(ConstructionBots.get_node(G, v)),
        subtitle_text="",
        title_scale=_node_type_check(ConstructionBots.get_node(G, v)
        ) ? ConstructionBots._title_text_scale(ConstructionBots.get_node(G, v)) : 0.45,
    )
)
display(plt)
