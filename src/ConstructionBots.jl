module ConstructionBots

# Utility libraries
using Dates
using JLD2
using Logging
using Parameters
using Printf
using ProgressMeter
using Random
using StatsBase
using TOML
# Handle arrays and mathematical operations
using DataStructures
using ForwardDiff
using JuMP, MathOptInterface
using LinearAlgebra
using SparseArrays
using StaticArrays
# Handle geometric data and graph structures
using CoordinateTransformations
using GeometryBasics
using Graphs, MetaGraphs
using LazySets
using Rotations
using SpatialIndexing
# Python interfacing, handling LDraw LEGO models
using Colors
using LDrawParser
using PyCall
# Optimization solvers
using ECOS, GLPK, Gurobi, HiGHS
# Generate graphical plots
using Compose, Measures, MeshCat

include("constants.jl")
include("utils/graph_utils.jl")
include("utils/taskgraphs_components.jl")
include("utils/hierarchical_geometry.jl")
include("schedule_construction.jl")
include("task_assignment.jl")
include("utils/render_tools.jl")
include("route_planning.jl")
include("utils/demo_utils.jl")

################################################################################
############################ Constructing Model Tree ###########################
################################################################################
# model::MPDModel - model.parts contains raw geometry of all parts
# assembly_tree::AssemblyTree - stored transforms of all parts and submodels
# model_schedule - encodes the partial ordering of assembly operations.

export run_demo, list_projects, get_project_params

"""
run_demo(; kwargs...)

Run the demo end-to-end.

Keyword arguments:
- `ldraw_file::String`: ldraw file
- `project_name::String`: project name (default: ldraw_file)
- `model_scale::Float64`: scale of the model (default: 0.008)
- `num_robots::Int`: number of robots to use (default: 12)
- `robot_scale::Float64`: scale of the robots (default: 0.7 * model_scale)
- `robot_height::Float64`: height of the robots (default: 10 * robot_scale)
- `robot_radius::Float64`: radius of the robots (default: 25 * robot_scale)
- `num_object_layers::Int`: number of layers to stack the legos (default: 1)
- `max_steps::Int`: maximum number of steps to run the simulation (default: 100000)
- `staging_buffer_factor::Float64`: factor to multiply the largest transport unit radius by to get the staging buffer (default: 1.2)
- `build_step_buffer_factor::Float64`: factor to multiply the robot radius by to get the build step buffer (default: 0.5)
- `base_results_path::String`: base path to save results to
- `results_path::String`: folder name to save the results to (inside base_results_path)
- `assignment_mode::Symbol`: assignment mode to use (default: :greedy, options: :greedy, :milp, :milp_w_greedy_warm_start)
- `open_animation_at_end::Bool`: open the animation in the browser at the end of the simulation (default: false)
- `save_animation::Bool`: save the animation at the end of the simulation (default: false)
- `save_animation_along_the_way::Bool`: save the animation at periodic intervals during the simulation (default: false)
- `anim_prog_path::String`: the file path or directory where the animation progress files should be saved 
- `anim_active_agents::Bool`: animate which agents are active (green circles) (default: false)
- `anim_active_areas::Bool`: animate which areas are active (purple circles) (default: false)
- `process_updates_interval::Int`: the interval to process animation updates (default: 25)
- `block_save_anim::Bool`: whether to save the animation in blocks instead of incrementally. false = incrementall, true = save in blocks (default: false)
- `update_anim_at_every_step::Bool`: whether to update the animation at every step (default: false)
- `save_anim_interval::Int`: the interval of number of updates to save the animation if `save_animation_along_the_way=true` (default: 500)
- `deconflict_strategies::Vector{Symbol}`: algorithm(s) used for decentralized collision avoidance (:RVO, :TangentBugPolicy, :PotentialFields)
- `overwrite_results::Bool`: whether to overwrite the stats.toml file or create a new one with a date-time filename (default: true)
- `write_results::Bool`: whether to write the results to disk (default: true)
- `max_num_iters_no_progress::Int`: maximum number of iterations to run without progress (default: 10000)
- `sim_batch_size::Int`: number of steps to run in a simulation before clearing some memory (default: 50)
- `log_level::Logging.LogLevel`: log level to use (default: Logging.Warn)
- `milp_optimizer::Symbol`: optimizer to use for the MILP (default: :highs, options: :gurobi, :glpk)
- `milp_optimizer_attribute_dict::Dict`: dictionary of attributes to set on the optimizer (default: Dict())
- `optimizer_time_limit::Int`: time limit for the optimizer (default: 600)
- `look_for_previous_milp_solution::Bool`: whether to look for a previous MILP solution (default: false)
- `save_milp_solution::Bool`: whether to save the MILP solution (default: false)
- `previous_found_optimizer_time::Int`: time limit for the optimizer when looking for a previous solution (default: 30)
- `stop_after_task_assignment::Bool`: whether to stop after task assignment (default: false)
- `ignore_rot_matrix_warning::Bool`: whether to ignore the rotation matrix warning (default: true)
- `rng::Random.AbstractRNG`: random number generator to use (default: MersenneTwister(1))
"""
function run_demo(;
    ldraw_file::String = "tractor.mpd",
    project_name::String = ldraw_file,
    model_scale::Float64 = 0.008,
    num_robots::Int = 12,
    robot_scale::Float64 = model_scale * 0.7,
    robot_height::Float64 = 10 * robot_scale,
    robot_radius::Float64 = 25 * robot_scale,
    num_object_layers::Int = 1,
    max_steps::Int = 100000,
    staging_buffer_factor::Float64 = 1.2,
    build_step_buffer_factor::Float64 = 0.5,
    base_results_path::String = joinpath(
        dirname(pathof(ConstructionBots)),
        "..",
        "results",
    ),
    results_path::String = joinpath(base_results_path, project_name),
    assignment_mode::Symbol = :greedy,
    open_animation_at_end::Bool = false,
    save_animation::Bool = false,
    save_animation_along_the_way::Bool = false,
    anim_active_agents::Bool = false,
    anim_active_areas::Bool = false,
    process_updates_interval::Int = 50,
    block_save_anim::Bool = false,
    update_anim_at_every_step::Bool = false,
    save_anim_interval::Int = 500,
    deconflict_strategies::Vector{Symbol} = [:Nothing],
    overwrite_results::Bool = false,
    write_results::Bool = true,
    max_num_iters_no_progress::Int = 10000,
    sim_batch_size::Int = 50,
    log_level::Logging.LogLevel = Logging.Warn,
    milp_optimizer::Symbol = :highs,
    milp_optimizer_attribute_dict::Dict = Dict(),
    optimizer_time_limit::Int = 600,
    look_for_previous_milp_solution::Bool = false,
    save_milp_solution::Bool = false,
    previous_found_optimizer_time::Int = 30,
    stop_after_task_assignment::Bool = false,
    ignore_rot_matrix_warning::Bool = true,
    rng::Random.AbstractRNG = Random.MersenneTwister(1),
)
    process_animation_tasks =
        save_animation || save_animation_along_the_way || open_animation_at_end
    # TODO(tashakim): create a helper method to check individual and combined
    # policies as a DeconflictStrategy subtype.
    deconfliction_type = ReciprocalVelocityObstacle()
    # deconfliction_type = if haskey(supported_deconfliction_options, deconflict_strategies)
    #     supported_deconfliction_options[deconflict_strategies]
    # else
    #     Nothing
    # end
    if deconfliction_type isa ReciprocalVelocityObstacle && !(deconfliction_type isa PotentialFields)
        @warn "RVO is enabled but potential fields is disabled. This is not recommended."
    end
    if block_save_anim
        if process_updates_interval != save_anim_interval
            @warn "When block_save_anim is true, it is recommended to set save_anim_interval to the same value as process_updates_interval."
        end
        if !save_animation_along_the_way
            @warn "block_save_anim is true but save_animation_along_the_way is false. Will save along the way anyway."
        end
    end
    # Record statistics
    stats = Dict()
    stats[:rng] = "$rng"
    stats[:modelscale] = model_scale
    stats[:robotscale] = robot_scale
    stats[:assignment_mode] = string(assignment_mode)
    stats[:deconfliction_type] = string(deconfliction_type.name)
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
    # Add additional attributes for GLPK, HiGHS, and Gurobi
    time_limit_key = nothing
    if assignment_mode == :milp || assignment_mode == :milp_w_greedy_warm_start
        milp_optimizer_attribute_dict[MOI.Silent()] = false
        default_milp_optimizer = nothing
        if milp_optimizer == :glpk
            default_milp_optimizer =
                () -> GLPK.Optimizer(; want_infeasibility_certificates = false)
            milp_optimizer_attribute_dict["tm_lim"] = optimizer_time_limit * 1000
            milp_optimizer_attribute_dict["msg_lev"] = GLPK.GLP_MSG_ALL
            time_limit_key = "tm_lim"
        elseif milp_optimizer == :gurobi
            default_milp_optimizer = () -> Gurobi.Optimizer()
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

    # Generate file name and paths based on the specified strategies and modes
    function generate_prefix(strategy_symbol, positive_prefix, negative_prefix, strategies)
        return supported_deconfliction_options[strategy_symbol] == strategies ? positive_prefix : negative_prefix
    end

    function get_solution_prefix(mode)
        if mode == :milp
            return "milp_"
        elseif mode == :greedy
            return "greedy_"
        elseif mode == :milp_w_greedy_warm_start
            return "milp-ws_"
        else
            error("Unknown assignment mode: $(mode)")
        end
    end
    function generate_file_name(base_name, overwrite_results)
        return overwrite_results ? base_name :
               string(Dates.format(Dates.now(), "yyyymmdd_HHMMSS"), "_", base_name)
    end
    prefix = string(
        generate_prefix(:RVO, "RVO", "no-RVO", deconflict_strategies),
        "_",
        generate_prefix(:PotentialFields, "PotentialFields", "no-PotentialFields", deconflict_strategies),
        "_",
        generate_prefix(
            :TangentBugPolicy,
            "TangentBug",
            "no-TangentBug",
            deconflict_strategies,
        ),
    )
    soln_str_pre = get_solution_prefix(assignment_mode)
    prefix = string(soln_str_pre, prefix)
    mkpath(joinpath(results_path, prefix))
    stats_path =
        joinpath(results_path, prefix, generate_file_name("stats.toml", overwrite_results))
    anim_path = joinpath(
        results_path,
        prefix,
        generate_file_name("visualization.html", overwrite_results),
    )
    anim_prog_path = joinpath(
        results_path,
        prefix,
        generate_file_name("visualization_", overwrite_results),
    )
    sim_params = SimParameters(
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
        max_num_iters_no_progress,
    )
    reset_all_id_counters!()
    reset_all_invalid_id_counters!()
    set_default_robot_geom!(
        Cylinder(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, robot_height), robot_radius),
    )
    # TODO(tashakim): Move logic below into deconfliction interface.
    ConstructionBots.set_default_loading_speed!(50 * default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(50 * default_robot_radius())
    ConstructionBots.set_staging_buffer_radius!(default_robot_radius()) # for tangent_bug policy
    set_agent_properties(deconfliction_type)
    # Set default optimizer for staging layout
    ConstructionBots.set_default_geom_optimizer!(ECOS.Optimizer)
    ConstructionBots.set_default_geom_optimizer_attributes!(MOI.Silent() => true)

    pre_execution_start_time = time()
    model =
        parse_ldraw_file(filename; ignore_rotation_determinant = ignore_rot_matrix_warning)
    populate_part_geometry!(model; ignore_rotation_determinant = ignore_rot_matrix_warning)
    LDrawParser.change_coordinate_system!(
        model,
        ldraw_base_transform(),
        model_scale;
        ignore_rotation_determinant = ignore_rot_matrix_warning,
    )
    # Construct Model Spec
    print("Constructing model spec...")
    spec = ConstructionBots.construct_model_spec(model)
    model_spec = ConstructionBots.extract_single_model(spec)
    id_map = ConstructionBots.build_id_map(model, model_spec)
    color_map = ConstructionBots.construct_color_map(model_spec, id_map)
    @assert validate_graph(model_spec)
    print("done!\n")
    ## Construct SceneTree
    print("Constructing scene tree...")
    assembly_tree = ConstructionBots.construct_assembly_tree(model, model_spec, id_map)
    scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
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
    ConstructionBots.init_transport_units!(scene_tree; robot_radius = robot_radius)
    config_transport_units_time = time() - config_transport_units_time
    print("done!\n")
    # Validate SceneTree
    print("Validating scene tree...")
    root = get_node(scene_tree, collect(get_all_root_nodes(scene_tree))[1])
    validate_tree(get_transform_node(root))
    validate_embedded_tree(scene_tree, v -> get_transform_node(get_node(scene_tree, v)))
    print("done!\n")
    # Add robots to scene tree
    robot_spacing = 5 * robot_radius
    robot_start_box_side = ceil(sqrt(num_robots)) * robot_spacing
    xy_range = (-robot_start_box_side/2:robot_spacing:robot_start_box_side/2)
    vtxs = ConstructionBots.construct_vtx_array(;
        spacing = (1.0, 1.0, 0.0),
        ranges = (xy_range, xy_range, 0:0),
    )
    robot_vtxs = StatsBase.sample(rng, vtxs, num_robots; replace = false)
    ConstructionBots.add_robots_to_scene!(scene_tree, robot_vtxs, [default_robot_geom()])
    # Recompute approximate geometry for when the robot is transporting it
    # Add temporary robots to the transport units and recalculate the bounding geometry
    # then remove them after the new geometries are calcualted
    ConstructionBots.add_temporary_invalid_robots!(scene_tree; with_edges = true)
    compute_approximate_geometries!(scene_tree, HypersphereKey())
    @assert all(
        map(
            node -> has_vertex(node.geom_hierarchy, HypersphereKey()),
            get_nodes(scene_tree),
        ),
    )
    compute_approximate_geometries!(scene_tree, HyperrectangleKey())
    @assert all(
        map(
            node -> has_vertex(node.geom_hierarchy, HyperrectangleKey()),
            get_nodes(scene_tree),
        ),
    )
    ConstructionBots.remove_temporary_invalid_robots!(scene_tree)
    # Construct Partial Schedule (without robots assigned)
    print("Constructing partial schedule...")
    jump_to_final_configuration!(scene_tree; set_edges = true)
    sched = construct_partial_construction_schedule(model, model_spec, scene_tree, id_map)
    @assert validate_schedule_transform_tree(sched)
    print("done!\n")
    # Generate staging plan
    print("Generating staging plan...")
    max_object_transport_unit_radius =
        ConstructionBots.get_max_object_transport_unit_radius(scene_tree)
    staging_plan_time = time()
    staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(
        scene_tree,
        sched;
        buffer_radius = staging_buffer_factor * max_object_transport_unit_radius,
        build_step_buffer_radius = build_step_buffer_factor * default_robot_radius(),
    )
    staging_plan_time = time() - staging_plan_time
    print("done!\n")
    # Record statistics
    stats[:numobjects] = length([
        node_id(n) for n in get_nodes(scene_tree) if matches_template(ObjectNode, n)
    ])
    stats[:numassemblies] = length([
        node_id(n) for n in get_nodes(scene_tree) if matches_template(AssemblyNode, n)
    ])
    stats[:numrobots] = length([
        node_id(n) for n in get_nodes(scene_tree) if matches_template(RobotNode, n)
    ])
    stats[:ConfigTransportUnitsTime] = config_transport_units_time
    stats[:StagingPlanTime] = staging_plan_time
    if write_results
        open(stats_path, "w") do io
            TOML.print(io, stats)
        end
    end
    # Move objects to the starting locations. Keep expanding a square from the origin until
    # all objects are placed while not putting obhects in building locations.
    print("Placing objects at starting locations ...")
    # Max height of cargo (excluding the final assembly)
    max_cargo_height = maximum(
        map(
            n -> get_base_geom(n, HyperrectangleKey()).radius[3] * 2,
            filter(
                n -> (
                    matches_template(TransportUnitNode, n) && cargo_id(n) != AssemblyID(1)
                ),
                get_nodes(scene_tree),
            ),
        ),
    )
    other_circles = get_buildstep_circles(sched)
    build_circles = remove_redundant(collect(values(other_circles)); ϵ = robot_radius)
    object_vtxs = get_object_vtx(
        scene_tree,
        build_circles,
        max_cargo_height,
        num_object_layers,
        2 * robot_radius,
    )
    ConstructionBots.select_initial_object_grid_locations!(sched, object_vtxs)
    # Move assemblies up so they float above the robots
    for node in get_nodes(scene_tree)
        if matches_template(AssemblyNode, node)
            start_node = get_node(sched, AssemblyComplete(node))
            # raise start
            current = global_transform(start_config(start_node))
            rect = current(get_base_geom(node, HyperrectangleKey()))
            dh = max_cargo_height - (rect.center.-rect.radius)[3]
            set_desired_global_transform_without_affecting_children!(
                start_config(start_node),
                CoordinateTransformations.Translation(current.translation[1:2]..., dh) ∘
                CoordinateTransformations.LinearMap(current.linear),
            )
        end
    end
    # Make sure all transforms line up
    ConstructionBots.calibrate_transport_tasks!(sched)
    @assert validate_schedule_transform_tree(sched; post_staging = true)
    print("done!\n")
    # Task Assignments
    print("Assigning robots...")
    ConstructionBots.add_dummy_robot_go_nodes!(sched)
    @assert validate_schedule_transform_tree(sched; post_staging = true)
    # Convert to OperatingSchedule
    ConstructionBots.set_default_loading_speed!(50 * default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(50 * default_robot_radius())
    tg_sched = ConstructionBots.convert_to_operating_schedule(sched)
    assignment_time = time()
    ## MILP solver
    solution_fname = joinpath(results_path, string(soln_str_pre, "$(num_robots).jld2"))
    no_solution_fname =
        joinpath(results_path, string(soln_str_pre, "$(num_robots)_NO-SOLN.jld2"))
    soln_matrix = nothing
    if look_for_previous_milp_solution && (assignment_mode != :greedy)
        if milp_optimizer == :glpk
            @warn """
            GLPK is not currently implemented through JuMP to support warm-starting.
            Recommend using HiGHS (:highs) or Gurobi (:gurobi) when using previous solutions."""
        end
        if isfile(solution_fname)
            println("\n\tFound previous MILP solution at $solution_fname")
            soln_matrix = JLD2.load(solution_fname, "soln_matrix")
            soln_matrix = SparseArrays.SparseMatrixCSC(soln_matrix)
            milp_optimizer_attribute_dict[time_limit_key] =
                Float64(previous_found_optimizer_time)
            clear_default_milp_optimizer_attributes!()
            set_default_milp_optimizer_attributes!(milp_optimizer_attribute_dict)
        elseif isfile(no_solution_fname)
            println("Aborting based on finding $no_solution_fname")
            throw(NoSolutionError())
        else
            println("\n\tNo previous MILP solution at $solution_fname")
        end
    end
    valid_milp_solution = true
    milp_model = SparseAdjacencyMILP()
    if assignment_mode == :milp
        if !isnothing(soln_matrix)
            milp_model = formulate_milp(
                milp_model,
                tg_sched,
                scene_tree;
                warm_start_soln = soln_matrix,
            )
        else
            milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        end
        optimize!(milp_model)
        if primal_status(milp_model) == MOI.NO_SOLUTION
            valid_milp_solution = false
        end
    elseif assignment_mode == :greedy  ## greedy assignment with enforced build-step ordering
        milp_model =
            ConstructionBots.GreedyOrderedAssignment(greedy_cost = GreedyFinalTimeCost())
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        optimize!(milp_model)
    elseif assignment_mode == :milp_w_greedy_warm_start
        if milp_optimizer == :glpk
            println()
            @warn """
            GLPK is not currently implemented through JuMP to support warm-starting.
            Recommend using HiGHS (:highs) or Gurobi (:gurobi)."""
        end
        if !isnothing(soln_matrix)
            milp_model = formulate_milp(
                milp_model,
                tg_sched,
                scene_tree;
                warm_start_soln = soln_matrix,
            )
        else
            greedy_sched = deepcopy(tg_sched)
            greedy_model = ConstructionBots.GreedyOrderedAssignment(
                greedy_cost = GreedyFinalTimeCost(),
            )
            greedy_model = formulate_milp(greedy_model, greedy_sched, scene_tree)
            optimize!(greedy_model)
            greedy_assignmnet_matrix = get_assignment_matrix(greedy_model)

            milp_model = SparseAdjacencyMILP()
            milp_model = formulate_milp(
                milp_model,
                tg_sched,
                scene_tree;
                warm_start_soln = greedy_assignmnet_matrix,
            )
        end
        optimize!(milp_model)
        if primal_status(milp_model) == MOI.NO_SOLUTION
            valid_milp_solution = false
        end
    else
        error("Unknown assignment mode: $assignment_mode")
    end
    post_assignment_makespan = Inf
    if valid_milp_solution
        validate_schedule_transform_tree(
            ConstructionBots.convert_from_operating_schedule(typeof(sched), tg_sched);
            post_staging = true,
        )
        update_project_schedule!(nothing, milp_model, tg_sched, scene_tree)
        @assert validate(tg_sched)
        # Assign robots to "home" locations so they don't sit around in each others' way
        go_nodes = [
            n for n in get_nodes(tg_sched) if
            matches_template(RobotGo, n) && is_terminal_node(tg_sched, n)
        ]
        min_assembly_1_xy =
            (staging_circles[AssemblyID(1)].center.-staging_circles[AssemblyID(1)].radius)[1:2]
        for (ii, n) in enumerate(go_nodes)
            vtx_x = min_assembly_1_xy[1] - ii * 5 * robot_radius
            vtx_y = min_assembly_1_xy[2]
            set_desired_global_transform!(
                goal_config(n),
                CoordinateTransformations.Translation(vtx_x, vtx_y, 0.0) ∘
                identity_linear_map(),
            )
        end
        post_assignment_makespan = makespan(tg_sched)
    end
    assignment_time = time() - assignment_time
    print("done!\n")
    # If valid milp solution found, save if option was passed
    if valid_milp_solution && save_milp_solution && (assignment_mode != :greedy)
        println("Saving MILP solution to $solution_fname")
        JLD2.save(solution_fname, "soln_matrix", get_assignment_matrix(milp_model))
    elseif save_milp_solution && (assignment_mode != :greedy)
        println("Saving a no soltuion file to $no_solution_fname")
        JLD2.save(no_solution_fname, "soln_matrix", false)
    end
    # Compile pre execution statistics
    pre_execution_time = time() - pre_execution_start_time
    stats[:AssigmentTime] = assignment_time
    stats[:PreExecutionRuntime] = pre_execution_time
    stats[:OptimisticMakespan] = post_assignment_makespan
    stats[:ValidMILPSolution] = valid_milp_solution
    if write_results
        open(stats_path, "w") do io
            TOML.print(io, stats)
        end
    end
    if !valid_milp_solution
        throw(NoSolutionError())
    elseif stop_after_task_assignment
        return nothing, stats
    end
    factory_vis = ConstructionBots.FactoryVisualizer(vis = visualizer)
    if !isnothing(visualizer)
        delete!(visualizer)
        # Visualize assembly
        factory_vis = ConstructionBots.populate_visualizer!(
            scene_tree,
            visualizer;
            color_map = color_map,
            color = RGB(0.3, 0.3, 0.3),
            material_type = MeshLambertMaterial,
        )
        ConstructionBots.add_indicator_nodes!(factory_vis)
        factory_vis.staging_nodes = ConstructionBots.render_staging_areas!(
            visualizer,
            scene_tree,
            sched,
            staging_circles;
            dz = 0.00,
            color = RGBA(0.4, 0.0, 0.4, 0.5),
        )
        for (k, color) in [
            (HypersphereKey() => RGBA(0.0, 1.0, 0.0, 0.3)),
            (HyperrectangleKey() => RGBA(1.0, 0.0, 0.0, 0.3)),
        ]
            ConstructionBots.show_geometry_layer!(factory_vis, k; color = color)
        end
        for (k, nodes) in factory_vis.geom_nodes
            setvisible!(nodes, false)
        end
        setvisible!(factory_vis.geom_nodes[BaseGeomKey()], true)
        setvisible!(factory_vis.active_flags, false)
        set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges = true)
        ConstructionBots.update_visualizer!(factory_vis)
    end
    set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges = true)
    # Apply deconfliction strategies in simulation
    env = PlannerEnv(
        sched = tg_sched,
        scene_tree = scene_tree,
        staging_circles = staging_circles,
        max_robot_go_id = maximum([
            n.id.id for n in get_nodes(tg_sched) if matches_template(RobotGo, n)
        ]),
        max_cargo_id = maximum([
            cargo_id(entity(n)).id for
            n in get_nodes(tg_sched) if matches_template(TransportUnitGo, n)
        ]),
        deconflict_strategies = deconflict_strategies,
        deconfliction_type = deconfliction_type,
    )
    update_env_with_deconfliction(deconfliction_type, scene_tree, env)
    anim = nothing
    if process_animation_tasks
        print("Animating preprocessing step...")
        anim = ConstructionBots.AnimationWrapper(0)
        atframe(anim, ConstructionBots.current_frame(anim)) do
            jump_to_final_configuration!(scene_tree; set_edges = true)
            ConstructionBots.update_visualizer!(factory_vis)
            setvisible!(factory_vis.geom_nodes[HyperrectangleKey()], false)
            setvisible!(factory_vis.staging_nodes, false)
        end
        ConstructionBots.step_animation!(anim)
        ConstructionBots.animate_preprocessing_steps!(
            factory_vis,
            sched;
            dt = 0.0,
            anim = anim,
            interp_steps = 40,
        )
        setanimation!(visualizer, anim.anim, play = false)
        print("done\n")
        if save_animation_along_the_way
            save_animation!(visualizer, "$(anim_prog_path)preprocessing.html")
        end
    end
    execution_start_time = time()
    status, time_steps = run_simulation!(env, factory_vis, anim, sim_params)
    if status == true
        execution_time = time() - execution_start_time
    else
        execution_time = Inf
        time_steps = Inf
    end
    # Add results
    stats[:ExecutionRuntime] = execution_time
    stats[:Makespan] = time_steps * env.dt
    if write_results
        open(stats_path, "w") do io
            TOML.print(io, stats)
        end
    end
    if process_animation_tasks
        if save_animation
            save_animation!(visualizer, anim_path)
        end
        if open_animation_at_end
            open(visualizer)
        end
    end
    return env, stats
end

sample_projects = Dict(
    1 => :colored_8x8,                  # 33 x 1, 4 sec
    2 => :quad_nested,                  # 85 x 21, 26 sec
    3 => :heavily_nested,               # 1757 x 508, 62 min
    4 => :tractor,                      # 20 x 8, 2 sec
    5 => :tie_fighter,                  # 44 x 4, 7 sec
    6 => :x_wing_mini,                  # 61 x 12, 9 sec
    7 => :imperial_shuttle,             # 84 x 5, 13 sec
    8 => :x_wing_tie_mini,              # 105 x 17, 26 sec
    9 => :at_te_walker,                 # 100 x 22, 23 sec
    10 => :x_wing,                      # 309 x 28, 3 min
    11 => :passenger_plane,             # 326 x 28, 4 min
    12 => :imperial_star_destroyer,     # 418 x 11, 5 min
    13 => :kings_castle,                # 761 x 70, 21 min
    14 => :at_at,                       # 1105 x 2, ??
    15 => :saturn_v,                     # 1845 x 306, 163 min
)
project_parameters = Dict(
    :colored_8x8 => (
        project_name = "colored_8x8",
        file_name = "colored_8x8.ldr",
        model_scale = 0.008,
        num_robots = 24,
    ),
    :quad_nested => (
        project_name = "quad_nested",
        file_name = "quad_nested.mpd",
        model_scale = 0.0015,
        num_robots = 50,
    ),
    :heavily_nested => (
        project_name = "heavily_nested",
        file_name = "heavily_nested.mpd",
        model_scale = 0.0015,
        num_robots = 50,
    ),
    :tractor => (
        project_name = "tractor",
        file_name = "tractor.mpd",
        model_scale = 0.008,
        num_robots = 10,
    ),
    :tie_fighter => (
        project_name = "tie_fighter",
        file_name = "8028-1 - TIE Fighter - Mini.mpd",
        model_scale = 0.008,
        num_robots = 15,
    ),
    :x_wing_mini => (
        project_name = "x_wing_mini",
        file_name = "30051-1 - X-wing Fighter - Mini.mpd",
        model_scale = 0.008,
        num_robots = 20,
    ),
    :imperial_shuttle => (
        project_name = "imperial_shuttle",
        file_name = "4494-1 - Imperial Shuttle - Mini.mpd",
        model_scale = 0.008,
        num_robots = 20,
    ),
    :x_wing_tie_mini => (
        project_name = "x_wing_tie_mini",
        file_name = "X-wing--Tie Mini.mpd",
        model_scale = 0.008,
        num_robots = 20,
    ),
    :at_te_walker => (
        project_name = "at_te_walker",
        file_name = "20009-1 - AT-TE Walker - Mini.mpd",
        model_scale = 0.008,
        num_robots = 35,
    ),
    :x_wing => (
        project_name = "x_wing",
        file_name = "7140-1 - X-wing Fighter.mpd",
        model_scale = 0.004,
        num_robots = 50,
    ),
    :passenger_plane => (
        project_name = "passenger_plane",
        file_name = "3181 - Passenger Plane.mpd",
        model_scale = 0.004,
        num_robots = 50,
    ),
    :imperial_star_destroyer => (
        project_name = "imperial_star_destroyer",
        file_name = "8099-1 - Midi-Scale Imperial Star Destroyer.mpd",
        model_scale = 0.004,
        num_robots = 75,
    ),
    :kings_castle => (
        project_name = "kings_castle",
        file_name = "6080 - Kings Castle.mpd",
        model_scale = 0.004,
        num_robots = 125,
    ),
    :at_at => (
        project_name = "at_at",
        file_name = "75054-1 - AT-AT.mpd",
        model_scale = 0.004,
        num_robots = 150,
    ),
    :saturn_v => (
        project_name = "saturn_v",
        file_name = "21309-1 - NASA Apollo Saturn V.mpd",
        model_scale = 0.0015,
        num_robots = 200,
    ),
)

"""
    list_projects()

Prints a list of available projects.
"""
function list_projects()
    println("Available sample projects:")
    for ii in sort(collect(keys(sample_projects)))
        println("  $ii: $(sample_projects[ii])")
    end
end

"""
    get_project_params(project::Int)
    get_project_params(project::Symbol)

Returns the parameters for the project with the given number or symbol.
Returns a Dict with the following fields:
- `project_name` (String): The name of the project.
- `file_name` (String): The name of the LDraw file.
- `model_scale` (Float64): The default scale of the model.
- `num_robots` (Int): The default number of robots to use.
"""
function get_project_params(project::Int)
    if !(project in keys(sample_projects))
        list_projects()
        error("Project $project not found.")
    end
    return get_project_params(sample_projects[project])
end

function get_project_params(project::Symbol)
    if !(project in keys(project_parameters))
        list_projects()
        error("Project $project not found.")
    end
    return project_parameters[project]
end

"""
    BuildStepID <: AbstractID
"""
@with_kw struct BuildStepID <: AbstractID
    id::Int
end
"""
    SubModelPlanID <: AbstractID
"""
@with_kw struct SubModelPlanID <: AbstractID
    id::Int
end
"""
    SubFileRedID <: AbstractID
"""
@with_kw struct SubFileRefID <: AbstractID
    id::Int
end

"""
    DuplicateIDGenerator{K}

Generates duplicate IDs.
"""
struct DuplicateIDGenerator{K}
    id_counts::Dict{K,Int}
    id_map::Dict{K,K}
    DuplicateIDGenerator{K}() where {K} = new{K}(Dict{K,Int}(), Dict{K,K}())
end

_id_type(::DuplicateIDGenerator{K}) where {K} = K

function (g::DuplicateIDGenerator)(id)
    k = get!(g.id_map, id, id)
    g.id_counts[k] = get(g.id_counts, k, 0) + 1
    new_id = string(k, "-", string(g.id_counts[k]))
    g.id_map[new_id] = id
    new_id
end

function duplicate_subtree!(g, old_root, d = :out)
    vtxs = [get_vtx(g, old_root)]
    append!(vtxs, collect(map(e -> e.dst, edges(bfs_tree(g, old_root; dir = d)))))
    new_ids = Dict{Int,_id_type(g)}()
    for v in vtxs
        old_node = get_node(g, v)
        new_node = add_node!(g, node_val(old_node))
        new_ids[v] = node_id(new_node)
    end
    f = d == :out ? outneighbors : inneighbors
    for v in vtxs
        for vp in f(g, v)
            if d == :out
                add_edge!(g, new_ids[v], new_ids[vp])
            else
                add_edge!(g, new_ids[vp], new_ids[v])
            end
        end
    end
    return get_node(g, new_ids[get_vtx(g, old_root)])
end

"""
    MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}

Graph to represent the modeling operations required to build a LDraw model.
Currently used both as an assembly tree and a "model schedule".
In the model schedule, the final model is the root of the graph, and its
ancestors are the operations building up thereto.
"""
@with_kw_noshow struct MPDModelGraph{N,ID} <: AbstractCustomNDiGraph{CustomNode{N,ID},ID}
    graph::DiGraph = DiGraph()
    nodes::Vector{CustomNode{N,ID}} = Vector{CustomNode{N,ID}}()
    vtx_map::Dict{ID,Int} = Dict{ID,Int}()
    vtx_ids::Vector{ID} = Vector{ID}()  # maps vertex uid to actual graph node
    id_generator::DuplicateIDGenerator{ID} = DuplicateIDGenerator{ID}()
end

create_node_id(g, v::BuildingStep) = g.id_generator("BuildingStep")
create_node_id(g, v::SubModelPlan) =
    has_vertex(g, model_name(v)) ? g.id_generator(model_name(v)) : model_name(v)
create_node_id(g, v::SubFileRef) = g.id_generator(model_name(v))

function add_node!(g::MPDModelGraph{N,ID}, val::N) where {N,ID}
    id = create_node_id(g, val)
    add_node!(g, val, id)
end

"""
    add_build_step!(model_graph,build_step,parent=-1)

add a build step to the model_graph, and add edges from all children of the
parent step to the child.
        [   parent_step   ]
           |           |
        [input] ... [input]
           |           |
        [    build_step   ]
"""
function add_build_step!(model_graph, build_step::BuildingStep, preceding_step = -1)
    node = add_node!(model_graph, build_step)
    for line in build_step.lines
        input = add_node!(model_graph, line)
        add_edge!(model_graph, input, node)
        add_edge!(model_graph, preceding_step, input)
    end
    if has_vertex(model_graph, preceding_step) &&
       is_terminal_node(model_graph, preceding_step)
        add_edge!(model_graph, preceding_step, node)
    end
    node
end

function populate_model_subgraph!(model_graph, model::SubModelPlan)
    n = add_node!(model_graph, model)
    preceding_step = -1
    for build_step in model.steps
        if !isempty(build_step.lines)
            preceding_step = add_build_step!(model_graph, build_step, preceding_step)
        end
    end
    add_edge!(model_graph, preceding_step, n)
end

function construct_submodel_dependency_graph(model)
    g = NGraph{DiGraph,SubModelPlan,String}()
    for (k, m) in model.models
        n = add_node!(g, m, k)
    end
    for (k, m) in model.models
        for s in m.steps
            for line in s.lines
                if has_vertex(g, model_name(line))
                    add_edge!(g, model_name(line), k)
                end
            end
        end
    end
    return g
end

"""
Copy all submodel trees into the trees of their parent models.
"""
function copy_submodel_trees!(sched, model)
    sub_model_dependencies = construct_submodel_dependency_graph(model)
    for vp in topological_sort_by_dfs(sub_model_dependencies)
        k = get_vtx_id(sub_model_dependencies, vp)
        @assert has_vertex(sched, k) "SubModelPlan $k isn't in sched, but should be"
        for v in Graphs.vertices(sched)
            node = get_node(sched, v)
            val = node_val(node)
            if isa(val, SubFileRef)
                if model_name(val) == k
                    sub_model_plan = duplicate_subtree!(sched, k, :in)
                    add_edge!(sched, sub_model_plan, v)  # add before instead of replacing
                end
            end
        end
    end
    sched
end

"""
    update_build_step_parents!(model_spec)

Ensure that all `BuildingStep` nodes have the correct parent (submodel) name.
"""
function update_build_step_parents!(model_spec)
    descendant_map = backup_descendants(model_spec, n -> matches_template(SubModelPlan, n))
    for v in Graphs.vertices(model_spec)
        node = get_node(model_spec, v)
        if matches_template(BuildingStep, node)
            node = replace_node!(
                model_spec,
                BuildingStep(node_val(node), descendant_map[node_id(node)]),
                node_id(node),
            )
            @assert node_val(node).parent == descendant_map[node_id(node)]
        end
    end
    model_spec
end

"""
    construct_model_spec(model)

Edges go forward in time.
"""
function construct_model_spec(model)
    NODE_VAL_TYPE = Union{SubModelPlan,BuildingStep,SubFileRef}
    spec = MPDModelGraph{NODE_VAL_TYPE,String}()
    for (k, m) in model.models
        populate_model_subgraph!(spec, m)
    end
    copy_submodel_trees!(spec, model)
    update_build_step_parents!(spec)
    return spec
end

"""
    extract_single_model(sched::S,model_key) where {S<:MPDModelGraph}

From a model schedule with (potentially) multiple distinct models, extract just
the model graph with root id `model_key`.
"""
function extract_single_model(
    spec::S,
    model_key = get_vtx_id(spec, get_biggest_tree(spec)),
) where {S<:MPDModelGraph}
    @assert has_vertex(spec, model_key)
    new_spec = S(id_generator = spec.id_generator)
    root = get_vtx(spec, model_key)
    add_node!(new_spec, get_node(spec, root), get_vtx_id(spec, root))
    for v in reverse(topological_sort_by_dfs(spec))
        dst_id = get_vtx_id(spec, v)
        if !has_vertex(new_spec, dst_id)
            continue
        end
        if !has_vertex(new_spec, dst_id)
            transplant!(new_spec, spec, dst_id)
        end
        for v2 in inneighbors(spec, dst_id)
            src_id = get_vtx_id(spec, v2)
            if !has_vertex(new_spec, src_id)
                transplant!(new_spec, spec, src_id)
            end
            add_edge!(new_spec, src_id, dst_id)
        end
    end
    new_spec
end

# Edges for Project Spec
# TODO: Dispatch on graph type
validate_edge(::SubModelPlan, ::SubFileRef) = true
validate_edge(::BuildingStep, ::SubModelPlan) = true
validate_edge(::BuildingStep, ::BuildingStep) = true
validate_edge(::BuildingStep, ::SubFileRef) = true
validate_edge(::SubFileRef, ::BuildingStep) = true

eligible_successors(::SubFileRef) = Dict(BuildingStep => 1)
eligible_predecessors(::SubFileRef) = Dict(BuildingStep => 1, SubModelPlan => 1)
required_successors(::SubFileRef) = Dict(BuildingStep => 1)
required_predecessors(::SubFileRef) = Dict()

eligible_successors(::SubModelPlan) = Dict(SubFileRef => 1)
eligible_predecessors(::SubModelPlan) = Dict(BuildingStep => 1)
required_successors(::SubModelPlan) = Dict()
required_predecessors(::SubModelPlan) = Dict(BuildingStep => 1)

eligible_successors(::BuildingStep) =
    Dict(SubFileRef => typemax(Int), SubModelPlan => 1, BuildingStep => 1)
eligible_predecessors(n::BuildingStep) =
    Dict(SubFileRef => LDrawParser.n_lines(n), BuildingStep => 1)
required_successors(::BuildingStep) = Dict(Union{SubModelPlan,BuildingStep} => 1)
required_predecessors(n::BuildingStep) = Dict(SubFileRef => LDrawParser.n_lines(n))

"""
    construct_assembly_graph(model)

Construct an assembly graph, where each `SubModelPlan` has an outgoing edge to
each `SubFileRef` pointing to one of its components.
"""
function construct_assembly_graph(model)
    NODE_VAL_TYPE = Union{SubModelPlan,SubFileRef}
    model_graph = MPDModelGraph{NODE_VAL_TYPE,String}()
    for (k, m) in model.models
        n = add_node!(model_graph, m, k)
        for s in m.steps
            for line in s.lines
                np = add_node!(model_graph, line) #,id_generator(line.file))
                add_edge!(model_graph, n, np)
            end
        end
    end
    return model_graph
end

geom_node(m::DATModel) = GeomNode(LDrawParser.extract_geometry(m))
geom_node(m::SubModelPlan) = GeomNode(nothing)

function geom_node(model::MPDModel, ref::SubFileRef)
    if has_model(model, ref.file)
        return geom_node(get_model(model, ref.file))
    elseif has_part(model, ref.file)
        return geom_node(get_part(model, ref.file))
    end
    throw(ErrorException("Referenced file $(ref.file) is not in model"))
    # @warn "Referenced file $(ref.file) is not in model"
    GeomNode(nothing)
end

"""
    build_id_map(model::MPDModel,spec::MPDModelGraph)

Constructs a `Dict` mapping from `AbstractID <=> String` to keep track of the
correspondence between ids in different graphs. Only the following id types are
mapped:
- ObjectID <=> SubFileRef (if the ref points to a model, not a part)
- AssemblyID <=> SubModelPlan
"""
function build_id_map(model::MPDModel, spec::MPDModelGraph)
    id_map = Dict{Union{String,AbstractID},Union{String,AbstractID}}()
    for (v, node) in enumerate(get_nodes(spec))
        val = node_val(node)
        new_id = nothing
        if isa(val, SubFileRef)
            if LDrawParser.has_model(model, val.file)
                # new_id = get_unique_id(AssemblyID)
            elseif LDrawParser.has_part(model, val.file)
                new_id = get_unique_id(ObjectID)
            else
                continue
            end
        elseif isa(val, SubModelPlan)
            new_id = get_unique_id(AssemblyID)
        end
        if !(new_id === nothing)
            id_map[new_id] = node_id(node)
            id_map[node_id(node)] = new_id
        end
    end
    id_map
end

"""
    get_referenced_component(model_spec,scene_tree,id_map,node)

A hacky utility for retrieving the component added to an assembly by
node::CustomNode{SubFileRef,...}. Currently necessary because some
SubFileRef nodes reference an object (id_map[object_id] <=> id_map[ref_id]),
whereas other SubFileRef nodes reference the assembly encoded by their direct parent.
"""
function get_referenced_component(model_spec, id_map, node)
    @assert matches_template(SubFileRef, node)
    for v in inneighbors(model_spec, node)
        input = get_node(model_spec, v)
        if matches_template(SubModelPlan, input)
            return id_map[get_vtx_id(model_spec, v)]
        end
    end
    return get(id_map, node_id(node), nothing)
end

"""
    get_build_step_components(model_spec,id_map,step)

Given step::CustomNode{BuildingStep,...}, return the set of AbstractIDs pointing
to all of the components to be added to the parent assembly at that building
step.
"""
function get_build_step_components(model_spec, id_map, step)
    @assert matches_template(BuildingStep, step)
    part_ids = Set{Union{AssemblyID,ObjectID}}()
    for v in inneighbors(model_spec, step)
        child = get_node(model_spec, v)
        if matches_template(SubFileRef, child)
            push!(part_ids, get_referenced_component(model_spec, id_map, child))
        end
    end
    return part_ids
end

"""
    construct_assembly_tree(model::MPDModel,spec::MPDModelGraph,

Construct an assembly_tree::NTree{SceneNode,AbstractID}, a precursor to
SceneTree.
"""
function construct_assembly_tree(
    model::MPDModel,
    spec::MPDModelGraph,
    id_map = build_id_map(model, spec),
)
    assembly_tree = NTree{SceneNode,AbstractID}()
    parent_map = backup_descendants(spec, n -> matches_template(SubModelPlan, n))
    for v in reverse(topological_sort_by_dfs(spec))
        node = get_node(spec, v)
        id = node_id(node)
        haskey(id_map, id) ? nothing : continue
        new_id = id_map[id]
        val = node_val(node)
        ref = nothing
        parent_id = id_map[parent_map[id]]
        if isa(val, SubModelPlan)
            g = geom_node(val)
            add_node!(assembly_tree, AssemblyNode(new_id, g), new_id)
            is_terminal_node(spec, v) ? continue : nothing
            # Retrieve parent SubFileRef
            ref_node = get_node(spec, outneighbors(spec, v)[1])
            ref = node_val(ref_node)
            @assert isa(ref, SubFileRef) "ref is $(ref)"
            parent_id = id_map[parent_map[node_id(ref_node)]]
        elseif isa(val, SubFileRef)
            if has_model(model, val.file)
                # Do not add assembly here
                continue
            else
                has_part(model, val.file)
                # Add an object only
                @info "SUB FILE PART: $(node_id(node))"
                p = get_part(model, val.file)
                g = geom_node(p)
                add_node!(assembly_tree, ObjectNode(new_id, g), new_id)
                ref = val
            end
        elseif isa(val, BuildingStep)
            continue
        end
        @info "Attaching $(id_map[parent_id]) => $(id_map[new_id])"
        parent = node_val(get_node(assembly_tree, parent_id))
        t = LDrawParser.build_transform(ref)
        add_component!(parent, new_id => t)
        set_child!(assembly_tree, parent_id, new_id)
        @info "$(id_map[parent_id]) => $(id_map[new_id]) is $(has_edge(assembly_tree,parent_id,new_id))"
    end
    assembly_tree
end

"""
    convert_to_scene_tree(assembly_tree,set_children=true)

Convert an assembly tree to a `SceneTree`.
"""
function convert_to_scene_tree(assembly_tree; set_children::Bool = true)
    scene_tree = SceneTree()
    for n in get_nodes(assembly_tree)
        add_node!(scene_tree, node_val(n))
    end
    if set_children
        for e in edges(assembly_tree)
            src_id = get_vtx_id(assembly_tree, edge_source(e))
            dst_id = get_vtx_id(assembly_tree, edge_target(e))
            set_child!(scene_tree, src_id, dst_id)  # ensures that the full tree is correctly set up
        end
    end
    return scene_tree
end

"""
    construct_spatial_index(scene_tree)

Construct a `SpatialIndexing.RTree` for efficiently finding neighbors of an
    object in the assembly.
"""
function construct_spatial_index(scene_tree, frontier = get_all_root_nodes(scene_tree))
    rtree = RTree{Float64,3}(Int, AbstractID, leaf_capacity = 10, branch_capacity = 10)
    jump_to_final_configuration!(scene_tree)
    bounding_rects = Dict{AbstractID,SpatialIndexing.Rect}()
    for v in BFSIterator(scene_tree, frontier)
        node = get_node(scene_tree, v)
        geom = get_cached_geom(node, BaseGeomKey())
        if geom === nothing
            continue
        end
        pt_min = [1.0, 1.0, 1.0]
        pt_max = -1 * pt_min
        for pt in coordinates(geom)
            for i = 1:3
                pt_min[i] = min(pt[i], pt_min[i])
                pt_max[i] = max(pt[i], pt_max[i])
            end
        end
        rect = SpatialIndexing.Rect(
            (pt_min[1], pt_min[2], pt_min[3]),
            (pt_max[1], pt_max[2], pt_max[3]),
        )
        bounding_rects[node_id(node)] = rect
        insert!(rtree, rect, v, node_id(node))
    end
    rtree, bounding_rects
end

"""
    get_candidate_mating_parts(rtree,id)

Return the set of ids whose bounding rectangles overlap with
"""
function get_candidate_mating_parts(rtree, bounding_rects, id)
    rect = bounding_rects[id]
    neighbor_ids = Set{AbstractID}()
    for el in iterate(intersects_with(rtree, rect))
        push!(neighbor_ids, el.val)
    end
    return neighbor_ids
end

"""
    identify_closest_surfaces(geom,neighbor_geom,ϵ=1e-4)

Find the geometry elements (line, triangle, quadrilateral) of `geom` and
`neighbor_geom` that are within a distance `ϵ` of each other.
It is assumed that `geom` and `neighbor_geom` are both `Vector{GeometryBasics.Ngon}`
"""
function identify_closest_surfaces(geom, neighbor_geom)
    pairs = Vector{Tuple{Int,Int}}()
    for (i, a) in enumerate(geom)
        for (j, b) in enumerate(neighbor_geom)
            d = distance(a, b)
            if d < ϵ
                push!(pairs, a, b)
            end
        end
    end
end

"""
    compute_separating_hyperplane(ptsA,ptsB)

Find the optimal separating hyperplane between two point sets
"""
function compute_separating_hyperplane(ptsA, ptsB, dim = 3)
    model = JuMP.Model(default_geom_optimizer())
    set_optimizer_attributes(model, default_geom_optimizer_attributes()...)
    @variable(model, x[1:dim])
    @constraint(model, [1.0; x] in SecondOrderCone())
    @variable(model, a >= 0)
    for pt in ptsA
        @constraint(model, a >= dot(x, pt))
    end
    @variable(model, b >= 0)
    for pt in ptsB
        @constraint(model, b <= dot(x, pt))
    end
    @constraint(model, a >= b)
    @objective(model, Min, a - b)
    optimize!(model)
    if !(primal_status(model) == MOI.FEASIBLE_POINT)
        @warn "Failed to compute optimal separating hyperplane"
    end
    return normalize(value.(x))
end

function GeometryBasics.decompose(
    ::Type{TriangleFace{Int}},
    n::GeometryBasics.Ngon{3,Float64,N,Point{3,Float64}},
) where {N}
    return SVector{N - 1,TriangleFace{Int}}(TriangleFace{Int}(1, i, i + 1) for i = 1:N-1)
end

function GeometryBasics.decompose(::Type{Point{3,Float64}}, n::GeometryBasics.Ngon)
    return n.points
end

GeometryBasics.faces(n::GeometryBasics.Ngon{3,Float64,4,Point{3,Float64}}) =
    [TriangleFace{Int}(1, 2, 3), TriangleFace{Int}(1, 3, 4)]
GeometryBasics.faces(n::GeometryBasics.Ngon{3,Float64,3,Point{3,Float64}}) =
    [TriangleFace{Int}(1, 2, 3)]

GeometryBasics.coordinates(v::AbstractVector) =
    collect(Base.Iterators.flatten(map(coordinates, v)))

function GeometryBasics.coordinates(
    v::AbstractVector{G},
) where {N,G<:GeometryBasics.Ngon{3,Float64,N,Point{3,Float64}}}
    vcat(map(coordinates, v)...)
end

function GeometryBasics.faces(
    v::AbstractVector{G},
) where {N,G<:GeometryBasics.Ngon{3,Float64,N,Point{3,Float64}}}
    vcat(map(i -> map(f -> f .+ ((i - 1) * N), faces(v[i])), 1:length(v))...)
end

function GeometryBasics.faces(v::AbstractVector{G}) where {G<:GeometryBasics.Ngon}
    face_vec = Vector{TriangleFace{Int}}()
    i = 0
    for element in v
        append!(face_vec, map(f -> f .+ i, faces(element)))
        i = face_vec[end].data[3]
    end
    return face_vec
end

end
