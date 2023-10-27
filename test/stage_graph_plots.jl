
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

using ConstructionBots

using HiGHS
using Gurobi
using ECOS

include("../scripts/project_params.jl")

project_params = get_project_params(4) # 4 = tractor


open_animation_at_end        = false
save_animation_along_the_way = false
save_animation_at_end        = false
anim_active_agents           = false
anim_active_areas            = false

rvo_flag                     = true
tangent_bug_flag             = true
dispersion_flag              = true
assignment_mode              = :greedy

write_results                = false
overwrite_results            = false

ldraw_file                   = project_params[:file_name]
project_name                 = project_params[:project_name]
model_scale                  = project_params[:model_scale]
num_robots                   = project_params[:num_robots]

assignment_mode              = assignment_mode
milp_optimizer               = :gurobi # :gurobi :highs
optimizer_time_limit         = 100

rvo_flag                     = rvo_flag
tangent_bug_flag             = tangent_bug_flag
dispersion_flag              = dispersion_flag

open_animation_at_end        = open_animation_at_end
save_animation               = save_animation_at_end
save_animation_along_the_way = save_animation_along_the_way
anim_active_agents           = anim_active_agents
anim_active_areas            = anim_active_areas

write_results                = write_results
overwrite_results            = overwrite_results

look_for_previous_milp_solution = false
save_milp_solution              = false
previous_found_optimizer_time   = 200

robot_scale::Float64                      =model_scale * 0.7
robot_height::Float64                     =10 * robot_scale
robot_radius::Float64                     =25 * robot_scale
num_object_layers::Int                    =1
max_steps::Int                            =100000
staging_buffer_factor::Float64            =1.2
build_step_buffer_factor::Float64         =0.5
base_results_path::String                 =joinpath(dirname(pathof(ConstructionBots)), "..", "results")
results_path::String                      =joinpath(base_results_path, project_name)
process_updates_interval::Int             =25
save_anim_interval::Int                   =500
max_num_iters_no_progress::Int            =10000
sim_batch_size::Int                       =50
log_level::Logging.LogLevel               =Logging.Warn
milp_optimizer_attribute_dict::Dict       =Dict()

ignore_rot_matrix_warning= true

rng::Random.AbstractRNG                   =Random.MersenneTwister(1)

process_animation_tasks = save_animation || save_animation_along_the_way || open_animation_at_end

if rvo_flag && !dispersion_flag
    @warn "RVO is enabled but dispersion is disabled. This is not recommended."
end

# record statistics
stats = Dict()
stats[:rng] = "$rng"
stats[:modelscale] = model_scale
stats[:robotscale] = robot_scale
stats[:assignment_mode] = string(assignment_mode)
stats[:rvo_flag] = rvo_flag
stats[:tangent_bug_flag] = tangent_bug_flag
stats[:dispersion_flag] = dispersion_flag
stats[:OptimizerTimeLimit] = optimizer_time_limit

if assignment_mode == :milp
    stats[:Optimizer] = string(milp_optimizer)
end

if save_animation_along_the_way
    save_animation = true
end

visualizer = nothing
if process_animation_tasks
    visualizer = MeshCat.Visualizer()
end

mkpath(results_path)
filename = joinpath(dirname(pathof(ConstructionBots)), "..", "LDraw_files", ldraw_file)
@assert ispath(filename) "File $(filename) does not exist."

global_logger(ConsoleLogger(stderr, log_level))

# Adding additional attributes for GLPK, HiGHS, and Gurobi
time_limit_key = nothing
if assignment_mode == :milp || assignment_mode == :milp_w_greedy_warm_start
    milp_optimizer_attribute_dict[MOI.Silent()] = false
    default_milp_optimizer = nothing
    if milp_optimizer == :glpk
        default_milp_optimizer = ()->GLPK.Optimizer(;want_infeasibility_certificates=false)
        milp_optimizer_attribute_dict["tm_lim"] = optimizer_time_limit * 1000
        milp_optimizer_attribute_dict["msg_lev"] = GLPK.GLP_MSG_ALL
        time_limit_key = "tm_lim"
    elseif milp_optimizer == :gurobi
        default_milp_optimizer = ()->Gurobi.Optimizer()
        # default_milp_optimizer = Gurobi.Optimizer
        milp_optimizer_attribute_dict["TimeLimit"] = optimizer_time_limit
        # MIPFocus: 1 -- feasible solutions, 2 -- optimal solutions, 3 -- bound
        milp_optimizer_attribute_dict["MIPFocus"] = 1
        time_limit_key = "TimeLimit"
    elseif milp_optimizer == :highs
        default_milp_optimizer = () -> HiGHS.Optimizer()
        milp_optimizer_attribute_dict["time_limit"] = Float64(optimizer_time_limit)
        milp_optimizer_attribute_dict["presolve"] = "on"
        time_limit_key = "time_limit"
    else
        @warn "No additional parameters for $milp_optimizer were set."
    end
    set_default_milp_optimizer!(default_milp_optimizer)
    clear_default_milp_optimizer_attributes!()
    set_default_milp_optimizer_attributes!(milp_optimizer_attribute_dict)
end

if rvo_flag
    prefix = "RVO"
else
    prefix = "no-RVO"
end
if dispersion_flag
    prefix = string(prefix, "_Dispersion")
else
    prefix = string(prefix, "_no-Dispersion")
end
if tangent_bug_flag
    prefix = string(prefix, "_TangentBug")
else
    prefix = string(prefix, "_no-TangentBug")
end
soln_str_pre = ""
if assignment_mode == :milp
    soln_str_pre = "milp_"
    prefix = string(soln_str_pre, prefix)
elseif assignment_mode == :greedy
    soln_str_pre = "greedy_"
    prefix = string(soln_str_pre, prefix)
elseif assignment_mode == :milp_w_greedy_warm_start
    soln_str_pre = "milp-ws_"
    prefix = string(soln_str_pre, prefix)
else
    error("Unknown assignment mode: $(assignment_mode)")
end
mkpath(joinpath(results_path, prefix))

name_augment = ""
if !overwrite_results
    name_augment = string(Dates.format(Dates.now(), "yyyymmdd_HHMMSS"), "_")
end

stats_file_name = string(name_augment, "stats", ".toml")
anim_file_name = string(name_augment, "visualization.html")
anim_prog_file_name = string(name_augment, "visualization_")

stats_path = joinpath(results_path, prefix, stats_file_name)
anim_path = joinpath(results_path, prefix, anim_file_name)
anim_prog_path = joinpath(results_path, prefix, anim_prog_file_name)


sim_params = SimParameters(
    sim_batch_size,
    max_steps,
    process_animation_tasks,
    save_anim_interval,
    process_updates_interval,
    anim_active_agents,
    anim_active_areas,
    anim_prog_path,
    save_animation_along_the_way,
    max_num_iters_no_progress
)

reset_all_id_counters!()
reset_all_invalid_id_counters!()

set_default_robot_geom!(
    Cylinder(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, robot_height), robot_radius)
)

ConstructionBots.set_rvo_default_time_step!(1 / 40.0)
ConstructionBots.set_default_loading_speed!(50 * default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(50 * default_robot_radius())
ConstructionBots.set_staging_buffer_radius!(default_robot_radius()) # for tangent_bug policy
ConstructionBots.set_rvo_default_neighbor_distance!(16 * default_robot_radius())
ConstructionBots.set_rvo_default_min_neighbor_distance!(10 * default_robot_radius())

# Setting default optimizer for staging layout
ConstructionBots.set_default_geom_optimizer!(ECOS.Optimizer)
ConstructionBots.set_default_geom_optimizer_attributes!(MOI.Silent()=>true)

pre_execution_start_time = time()
model = parse_ldraw_file(filename; ignore_rotation_determinant=ignore_rot_matrix_warning)
populate_part_geometry!(model; ignore_rotation_determinant=ignore_rot_matrix_warning)
LDrawParser.change_coordinate_system!(model, ldraw_base_transform(), model_scale; ignore_rotation_determinant=ignore_rot_matrix_warning)

## CONSTRUCT MODEL SPEC
print("Constructing model spec...")
spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec)
id_map = ConstructionBots.build_id_map(model, model_spec)
color_map = ConstructionBots.construct_color_map(model_spec, id_map)
@assert validate_graph(model_spec)
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
compute_approximate_geometries!(scene_tree, HypersphereKey())
compute_approximate_geometries!(scene_tree, HyperrectangleKey())
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
root = get_node(scene_tree, collect(get_all_root_nodes(scene_tree))[1])
validate_tree(get_transform_node(root))
validate_embedded_tree(scene_tree, v -> get_transform_node(get_node(scene_tree, v)))
print("done!\n")

## Add robots to scene tree
robot_spacing = 5 * robot_radius
robot_start_box_side = ceil(sqrt(num_robots)) * robot_spacing
xy_range = (-robot_start_box_side/2:robot_spacing:robot_start_box_side/2)
vtxs = ConstructionBots.construct_vtx_array(; spacing=(1.0, 1.0, 0.0), ranges=(xy_range, xy_range, 0:0))

robot_vtxs = StatsBase.sample(rng, vtxs, num_robots; replace=false)

ConstructionBots.add_robots_to_scene!(scene_tree, robot_vtxs, [default_robot_geom()])

## Recompute approximate geometry for when the robot is transporting it
# Add temporary robots to the transport units and recalculate the bounding geometry
# then remove them after the new geometries are calcualted
ConstructionBots.add_temporary_invalid_robots!(scene_tree; with_edges=true)
compute_approximate_geometries!(scene_tree, HypersphereKey())
@assert all(map(node -> has_vertex(node.geom_hierarchy, HypersphereKey()), get_nodes(scene_tree)))
compute_approximate_geometries!(scene_tree, HyperrectangleKey())
@assert all(map(node -> has_vertex(node.geom_hierarchy, HyperrectangleKey()), get_nodes(scene_tree)))
ConstructionBots.remove_temporary_invalid_robots!(scene_tree)

## Construct Partial Schedule (without robots assigned)
print("Constructing partial schedule...")
jump_to_final_configuration!(scene_tree; set_edges=true)
sched = construct_partial_construction_schedule(model, model_spec, scene_tree, id_map)
@assert validate_schedule_transform_tree(sched)
print("done!\n")

### Plot the schedule ###
_node_type_check(n) = matches_template((ObjectStart,AssemblyStart,AssemblyComplete,FormTransportUnit,TransportUnitGo,DepositCargo,LiftIntoPlace),n)

plt = ConstructionBots.display_graph(
    sched;
    grow_mode=:from_left,
    align_mode=:split_aligned,
    draw_node_function=(G,v)->ConstructionBots.draw_node(get_node(G,v);
        title_text= _node_type_check(get_node(G,v)
            ) ? string(ConstructionBots._title_string(get_node(G,v)),
                "$(get_id(node_id(get_node(G,v))))") : ConstructionBots._title_string(get_node(G,v)),
        subtitle_text="",
        title_scale = _node_type_check(get_node(G,v)
            ) ? ConstructionBots._title_text_scale(get_node(G,v)) : 0.45,
    ),
    pad=(0.0, 0.0)
);


## Generate staging plan
max_object_transport_unit_radius = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)

staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree, sched;
    buffer_radius=staging_buffer_factor * max_object_transport_unit_radius,
    build_step_buffer_radius=build_step_buffer_factor * default_robot_radius()
)

#### Plot the staging area ####
ConstructionBots.plot_staging_area(
    sched, scene_tree, staging_circles;
    save_file_name="staging_area.pdf",
    save_image=false
);


# Make sure all transforms line up
ConstructionBots.calibrate_transport_tasks!(sched)
@assert validate_schedule_transform_tree(sched; post_staging=true)

# Task Assignments
ConstructionBots.add_dummy_robot_go_nodes!(sched)
@assert validate_schedule_transform_tree(sched; post_staging=true)

ConstructionBots.set_default_loading_speed!(50 * default_robot_radius())
ConstructionBots.set_default_rotational_loading_speed!(50 * default_robot_radius())

tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

milp_model = SparseAdjacencyMILP()
milp_model = formulate_milp(milp_model, tg_sched, scene_tree)

optimize!(milp_model)

validate_schedule_transform_tree(
    ConstructionBots.convert_from_operating_schedule(typeof(sched), tg_sched)
    ; post_staging=true
)
update_project_schedule!(nothing, milp_model, tg_sched, scene_tree)
@assert validate(tg_sched)

# Plot the schedule with robots assigned
plt = ConstructionBots.display_graph(
    tg_sched;
    grow_mode=:from_left,
    align_mode=:split_aligned,
    draw_node_function=(G,v)->ConstructionBots.draw_node(get_node(G,v);
        title_text= _node_type_check(get_node(G,v)
            ) ? string(ConstructionBots._title_string(get_node(G,v)),
                "$(get_id(node_id(get_node(G,v))))") : ConstructionBots._title_string(get_node(G,v)),
        subtitle_text="",
        title_scale = _node_type_check(get_node(G,v)
            ) ? ConstructionBots._title_text_scale(get_node(G,v)) : 0.45,
    )
);
