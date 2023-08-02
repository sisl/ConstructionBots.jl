using Parameters
using Random
using StaticArrays
using LinearAlgebra
using StatsBase
using JLD2
using ProgressMeter
using Logging
using LazySets
using GeometryBasics
using CoordinateTransformations
using Rotations
using TOML
using Graphs
using JuMP
using PyCall
using MeshCat

using ConstructionBots

using LDrawParser
using TaskGraphs
using HierarchicalGeometry
using GraphUtils


# TODO: Remove unused plotting functions
using Plots
using Colors
using Printf
using PGFPlots
using PGFPlotsX
using Measures
import Cairo
using Compose

include("../deps/GraphPlottingBFS.jl")
include("../deps/FactoryRendering.jl")
include("../src/render_tools.jl")
include("../src/tg_render_tools.jl")
include("demo_utils.jl")

"""
    run_lego_demo(;kwargs...)

Run the lego demo for model.

    Keyword arguments:

        - project_name::String: ldraw file of the model to run
        - model_scale::Float64: scale of the model (default: 0.008)
        - num_robots::Int: number of robots to use (default: 12)
        - robot_scale::Float64: scale of the robots (default: 0.7 * model_scale)
        - robot_height::Float64: height of the robots (default: 10 * robot_scale)
        - robot_radius::Float64: radius of the robots (default: 25 * robot_scale)
        - num_object_layers::Int: number of layers to stack the legos (default: 1)
        - max_steps::Int: maximum number of steps to run the simulation (default: 100000)
        - staging_buffer_factor::Float64: factor to multiply the largest transport unit radius by to get the staging buffer (default: 1.2)
        - build_step_buffer_factor::Float64: factor to multiply the robot radius by to get the build step buffer (default: 0.5)
        - base_results_path::String: base path to save results to
        - results_path::String: folder name to save the results to (inside base_results_path)
        - assignment_mode::Symbol: assignment mode to use (default: :greedy, options: :greedy, :milp, :milp_w_greedy_warm_start)
        - open_animation_at_end::Bool: open the animation in the browser at the end of the simulation (default: false)
        - save_animation::Bool: save the animation at the end of the simulation (default: false)
        - save_animation_along_the_way::Bool: save the animation at periodic intervals during the simulation (default: false)
        - anim_active_agents::Bool: animate which agents are active (green circles) (default: false)
        - anim_active_areas::Bool: animate which areas are active (purple circles) (default: false)
        - process_updates_interval::Int: the interval to process animation updates (default: 25)
        - save_anim_interval::Int: the interval of number of updates to save the animation if `save_animation_along_the_way=true` (default: 500)
        - rvo_flag::Bool: whether to use RVO (default: true)
        - tangent_bug_flag::Bool: whether to use tangent bug (default: true)
        - dispersion_flag::Bool: whether to use dispersion (default: true)
        - overwrite_results::Bool: whether to overwrite the results if they already exist (default: true)
        - write_results::Bool: whether to write the results to disk (default: true)
        - max_num_iters_no_progress::Int: maximum number of iterations to run without progress (default: 10000)
        - sim_batch_size::Int: number of steps to run in a simulation before clearing some memory (default: 50)
        - log_level::Logging.LogLevel: log level to use (default: Logging.Warn)
        - milp_optimizer::Symbol: optimizer to use for the MILP (default: :highs, options: :gurobi, :glpk)
        - milp_optimizer_attribute_dict::Dict: dictionary of attributes to set on the optimizer (default: Dict())
        - optimizer_time_limit::Int: time limit for the optimizer (default: 600)
        - rng::Random.AbstractRNG: random number generator to use (default: MersenneTwister(1))

"""
function run_lego_demo(;
    project_name::String                      ="tractor.mpd",
    model_scale::Float64                      =0.008,
    num_robots::Int                           =12,
    robot_scale::Float64                      =model_scale * 0.7,
    robot_height::Float64                     =10 * robot_scale,
    robot_radius::Float64                     =25 * robot_scale,
    num_object_layers::Int                    =1,
    max_steps::Int                            =100000,
    staging_buffer_factor::Float64            =1.2,
    build_step_buffer_factor::Float64         =0.5,
    base_results_path::String                =joinpath(dirname(pathof(ConstructionBots)), "..", "results"),
    results_path::String                     =joinpath(base_results_path, project_name),
    assignment_mode::Symbol                   =:greedy,
    open_animation_at_end::Bool               =false,
    save_animation::Bool                      =false,
    save_animation_along_the_way::Bool        =false,
    anim_active_agents::Bool                  =false,
    anim_active_areas::Bool                   =false,
    process_updates_interval::Int             =25,
    save_anim_interval::Int                   =500,
    rvo_flag::Bool                            =true,
    tangent_bug_flag::Bool                    =true,
    dispersion_flag::Bool                     =true,
    overwrite_results::Bool                   =true,
    write_results::Bool                       =true,
    max_num_iters_no_progress::Int            =10000,
    sim_batch_size::Int                       =50,
    log_level::Logging.LogLevel               =Logging.Warn,
    milp_optimizer::Symbol                    =:highs,
    milp_optimizer_attribute_dict::Dict       =Dict(),
    optimizer_time_limit::Int                 =600,
    rng::Random.AbstractRNG                   =Random.MersenneTwister(1)
)

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
    filename = joinpath(dirname(pathof(ConstructionBots)), "..", "LDraw_files", project_name)
    @assert ispath(filename) "File $(filename) does not exist."

    global_logger(ConsoleLogger(stderr, log_level))

    # Adding additional attributes for GLPK, HiGHS, and Gurobi
    milp_optimizer_attribute_dict[MOI.Silent()] = false
    default_milp_optimizer = nothing
    if milp_optimizer == :glpk
        default_milp_optimizer = ()->GLPK.Optimizer(;want_infeasibility_certificates=false)
        milp_optimizer_attribute_dict["tm_lim"] = optimizer_time_limit * 1000
        milp_optimizer_attribute_dict["msg_lev"] = GLPK.GLP_MSG_ALL
    elseif milp_optimizer == :gurobi
        default_milp_optimizer = Gurobi.Optimizer
        milp_optimizer_attribute_dict["TimeLimit"] = optimizer_time_limit
        # MIPFocus: 1 -- feasible solutions, 2 -- optimal solutions, 3 -- bound
        milp_optimizer_attribute_dict["MIPFocus"] = 1
    elseif milp_optimizer == :highs
        default_milp_optimizer = () -> HiGHS.Optimizer()
        milp_optimizer_attribute_dict["time_limit"] = Float64(optimizer_time_limit)
        milp_optimizer_attribute_dict["presolve"] = "on"
    else
        @warn "No additional parameters for $milp_optimizer were set."
    end
    set_default_milp_optimizer!(default_milp_optimizer)
    TaskGraphs.clear_default_optimizer_attributes!()

    TaskGraphs.set_default_optimizer_attributes!(milp_optimizer_attribute_dict)

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
    if assignment_mode == :milp
        prefix = string("milp_", prefix)
    elseif assignment_mode == :greedy
        prefix = string("greedy_", prefix)
    elseif assignment_mode == :milp_w_greedy_warm_start
        prefix = string("milp-ws_", prefix)
    else
        error("Unknown assignment mode: $(assignment_mode)")
    end
    mkpath(joinpath(results_path, prefix))
    stats_path = joinpath(results_path, prefix, "stats.toml")
    if isfile(stats_path) && write_results && !overwrite_results
        @warn "Terminating because results are already compiled at $(stats_path)"
        return nothing
    end

    anim_path = joinpath(results_path, prefix, "construction_simulation.html")
    anim_prog_path = joinpath(results_path, prefix, "construction_simulation_")


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
    FactoryRendering.set_render_param!(:Radius, :Robot, robot_radius)
    ConstructionBots.set_rvo_default_time_step!(1 / 40.0)
    ConstructionBots.set_default_loading_speed!(50 * HierarchicalGeometry.default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(50 * HierarchicalGeometry.default_robot_radius())
    ConstructionBots.set_staging_buffer_radius!(HierarchicalGeometry.default_robot_radius()) # for tangent_bug policy
    ConstructionBots.set_rvo_default_neighbor_distance!(16 * HierarchicalGeometry.default_robot_radius())
    ConstructionBots.set_rvo_default_min_neighbor_distance!(10 * HierarchicalGeometry.default_robot_radius())


    pre_execution_start_time = time()
    model = parse_ldraw_file(filename)
    populate_part_geometry!(model)
    LDrawParser.change_coordinate_system!(model, ldraw_base_transform(), model_scale)

    ## CONSTRUCT MODEL SPEC
    print("Constructing model spec...")
    spec = ConstructionBots.construct_model_spec(model)
    model_spec = ConstructionBots.extract_single_model(spec)
    id_map = ConstructionBots.build_id_map(model, model_spec)
    color_map = construct_color_map(model_spec, id_map)
    @assert GraphUtils.validate_graph(model_spec)
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
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HypersphereKey())
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HyperrectangleKey())
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
    validate_tree(HierarchicalGeometry.get_transform_node(root))
    validate_embedded_tree(scene_tree, v -> HierarchicalGeometry.get_transform_node(get_node(scene_tree, v)))
    print("done!\n")

    ## Add robots to scene tree
    robot_spacing = 5 * robot_radius
    robot_start_box_side = ceil(sqrt(num_robots)) * robot_spacing
    xy_range = (-robot_start_box_side/2:robot_spacing:robot_start_box_side/2)
    vtxs = ConstructionBots.construct_vtx_array(; spacing=(1.0, 1.0, 0.0), ranges=(xy_range, xy_range, 0:0))

    robot_vtxs = StatsBase.sample(rng, vtxs, num_robots; replace=false)
    # robot_vtxs = draw_random_uniform(vtxs, num_robots)

    ConstructionBots.add_robots_to_scene!(scene_tree, robot_vtxs, [default_robot_geom()])

    ## Recompute approximate geometry for when the robot is transporting it
    # Add temporary robots to the transport units and recalculate the bounding geometry
    # then remove them after the new geometries are calcualted
    ConstructionBots.add_temporary_invalid_robots!(scene_tree; with_edges=true)
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HypersphereKey())
    @assert all(map(node -> has_vertex(node.geom_hierarchy, HypersphereKey()), get_nodes(scene_tree)))
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HyperrectangleKey())
    @assert all(map(node -> has_vertex(node.geom_hierarchy, HyperrectangleKey()), get_nodes(scene_tree)))
    ConstructionBots.remove_temporary_invalid_robots!(scene_tree)

    ## Construct Partial Schedule (without robots assigned)
    print("Constructing partial schedule...")
    HierarchicalGeometry.jump_to_final_configuration!(scene_tree; set_edges=true)
    sched = construct_partial_construction_schedule(model, model_spec, scene_tree, id_map)
    @assert validate_schedule_transform_tree(sched)
    print("done!\n")

    ## Generate staging plan
    print("Generating staging plan...")
    max_object_transport_unit_radius = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)
    staging_plan_time = time()
    staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree, sched;
        buffer_radius=staging_buffer_factor * max_object_transport_unit_radius,
        build_step_buffer_radius=build_step_buffer_factor * HierarchicalGeometry.default_robot_radius()
    )
    staging_plan_time = time() - staging_plan_time
    print("done!\n")

    # record statistics
    stats[:numobjects] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(ObjectNode, n)])
    stats[:numassemblies] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(AssemblyNode, n)])
    stats[:numrobots] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(RobotNode, n)])
    stats[:ConfigTransportUnitsTime] = config_transport_units_time
    stats[:StagingPlanTime] = staging_plan_time
    if write_results
        open(joinpath(results_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, stats)
        end
    end

    # Move objects to the starting locations. Keep expanding a square from the origin until
    # all objects are placed while not putting obhects in building locations.
    print("Placing objects at starting locations ...")
    # Max height of cargo (excluding the final assembly)
    max_cargo_height = maximum(map(n -> get_base_geom(n, HyperrectangleKey()).radius[3] * 2,
        filter(n -> (matches_template(TransportUnitNode, n) && cargo_id(n) != AssemblyID(1)),
        get_nodes(scene_tree))))
    other_circles = get_buildstep_circles(sched)
    build_circles = remove_redundant(collect(values(other_circles)); ϵ = robot_radius)
    object_vtxs = get_object_vtx(scene_tree, build_circles, max_cargo_height,
        num_object_layers, 2 * robot_radius)
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
                CoordinateTransformations.Translation(current.translation[1:2]..., dh) ∘ CoordinateTransformations.LinearMap(current.linear)
            )
        end
    end
    # Make sure all transforms line up
    ConstructionBots.calibrate_transport_tasks!(sched)
    @assert validate_schedule_transform_tree(sched; post_staging=true)
    print("done!\n")

    # Task Assignments
    print("Assigning robots...")
    ConstructionBots.add_dummy_robot_go_nodes!(sched)
    @assert validate_schedule_transform_tree(sched; post_staging=true)

    # Convert to OperatingSchedule
    ConstructionBots.set_default_loading_speed!(50 * HierarchicalGeometry.default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(50 * HierarchicalGeometry.default_robot_radius())

    tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

    assignment_time = time()
    ## MILP solver
    valid_milp_solution = true
    milp_model = SparseAdjacencyMILP()
    if assignment_mode == :milp
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        optimize!(milp_model)
        if primal_status(milp_model) == MOI.NO_SOLUTION
            valid_milp_solution = false
        end
    elseif assignment_mode == :greedy
        ## Greedy Assignment with enforced build-step ordering
        milp_model = ConstructionBots.GreedyOrderedAssignment(
            greedy_cost=TaskGraphs.GreedyFinalTimeCost(),
        )
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        optimize!(milp_model)
    elseif assignment_mode == :milp_w_greedy_warm_start
        if milp_optimizer == :glpk
            println()
            @warn """GLPK is not currently implemented through JuMP to support warm-starting.
                       Recommend using HiGHS (:highs) or Gurobi (:gurobi)."""
        end
        greedy_sched = deepcopy(tg_sched)
        greedy_model = ConstructionBots.GreedyOrderedAssignment(
            greedy_cost=TaskGraphs.GreedyFinalTimeCost(),
        )
        greedy_model = formulate_milp(greedy_model, greedy_sched, scene_tree)
        optimize!(greedy_model)
        greedy_assignmnet_matrix = get_assignment_matrix(greedy_model)

        milp_model = SparseAdjacencyMILP()
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree; warm_start_soln=greedy_assignmnet_matrix)
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
            ConstructionBots.convert_from_operating_schedule(typeof(sched), tg_sched)
            ; post_staging=true)
        update_project_schedule!(nothing, milp_model, tg_sched, scene_tree)
        @assert validate(tg_sched)

        # Assign robots to "home" locations so they don't sit around in each others' way
        go_nodes = [n for n in get_nodes(tg_sched) if matches_template(RobotGo, n) && is_terminal_node(tg_sched, n)]
        min_assembly_1_xy = (staging_circles[AssemblyID(1)].center .- staging_circles[AssemblyID(1)].radius)[1:2]
        for (ii, n) in enumerate(go_nodes)
            vtx_x = min_assembly_1_xy[1] - ii * 5 * robot_radius
            vtx_y = min_assembly_1_xy[2]
            HierarchicalGeometry.set_desired_global_transform!(goal_config(n),
                CoordinateTransformations.Translation(vtx_x, vtx_y, 0.0) ∘ identity_linear_map()
            )
        end
        post_assignment_makespan = TaskGraphs.makespan(tg_sched)
    end
    assignment_time = time() - assignment_time
    print("done!\n")

    # compile pre execution statistics
    pre_execution_time = time() - pre_execution_start_time

    stats[:AssigmentTime] = assignment_time
    stats[:PreExecutionRuntime] = pre_execution_time
    stats[:OptimisticMakespan] = post_assignment_makespan
    stats[:ValidMILPSolution] = valid_milp_solution
    stats[:OptimizerTimeLimit] = optimizer_time_limit
    if write_results
        open(joinpath(results_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, stats)
        end
    end

    if !valid_milp_solution
        throw(NoSolutionError())
    end

    factory_vis = FactoryVisualizer(vis=visualizer)
    if !isnothing(visualizer)
        delete!(visualizer)

        # Visualize assembly
        factory_vis = populate_visualizer!(scene_tree, visualizer;
            color_map=color_map,
            color=RGB(0.3, 0.3, 0.3),
            material_type=MeshLambertMaterial
        )
        add_indicator_nodes!(factory_vis)
        factory_vis.staging_nodes = render_staging_areas!(visualizer, scene_tree,
            sched, staging_circles;
            dz=0.00, color=RGBA(0.4, 0.0, 0.4, 0.5))
        for (k, color) in [
            (HypersphereKey() => RGBA(0.0, 1.0, 0.0, 0.3)),
            (HyperrectangleKey() => RGBA(1.0, 0.0, 0.0, 0.3))
        ]
            show_geometry_layer!(factory_vis, k; color=color)
        end
        for (k, nodes) in factory_vis.geom_nodes
            setvisible!(nodes, false)
        end
        setvisible!(factory_vis.geom_nodes[BaseGeomKey()], true)
        setvisible!(factory_vis.active_flags, false)
        set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges=true)
        update_visualizer!(factory_vis)
    end

    set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges=true)


    # rvo
    if rvo_flag
        ConstructionBots.reset_rvo_python_module!()
        ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(; horizon=2.0))
    end


    max_robot_go_id = maximum([n.id.id for n in get_nodes(tg_sched) if matches_template(RobotGo, n)])
    max_cargo_id = maximum([cargo_id(entity(n)).id for n in get_nodes(tg_sched) if matches_template(TransportUnitGo, n)])

    env = PlannerEnv(
        sched=tg_sched,
        scene_tree=scene_tree,
        staging_circles=staging_circles,
        max_robot_go_id=max_robot_go_id,
        max_cargo_id=max_cargo_id,
    )
    active_nodes = (get_node(tg_sched, v) for v in env.cache.active_set)

    if rvo_flag
        ConstructionBots.rvo_add_agents!(scene_tree, active_nodes)
    end

    static_potential_function = (x, r) -> 0.0
    pairwise_potential_function = ConstructionBots.repulsion_potential
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = HierarchicalGeometry.get_radius(get_base_geom(n, HypersphereKey()))
            vmax = ConstructionBots.get_rvo_max_speed(n)

            tagent_bug_pol = nothing
            dispersion_pol = nothing
            if tangent_bug_flag
                tagent_bug_pol = TangentBugPolicy(
                    dt=env.dt, vmax=vmax, agent_radius=agent_radius
                )
            end
            if dispersion_flag
                dispersion_pol = ConstructionBots.PotentialFieldController(
                    agent=n, node=node, agent_radius=agent_radius, vmax=vmax,
                    max_buffer_radius=2.5 * agent_radius,
                    static_potentials=static_potential_function,
                    pairwise_potentials=pairwise_potential_function
                )
            end

            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy=tagent_bug_pol,
                dispersion_policy=dispersion_pol
            )
        end
    end


    anim = nothing
    if process_animation_tasks
        print("Animating preprocessing step...")
        anim = AnimationWrapper(0)
        atframe(anim, current_frame(anim)) do
            HierarchicalGeometry.jump_to_final_configuration!(scene_tree; set_edges=true)
            update_visualizer!(factory_vis)
            setvisible!(factory_vis.geom_nodes[HyperrectangleKey()], false)
            setvisible!(factory_vis.staging_nodes, false)
        end
        step_animation!(anim)
        animate_preprocessing_steps!(
            factory_vis,
            sched;
            dt=0.0,
            anim=anim,
            interp_steps=40
        )
        setanimation!(visualizer, anim.anim, play=false)
        print("done\n")

        if save_animation_along_the_way
            save_animation!(visualizer, "$(anim_prog_path)preprocessing.html")
        end

    end

    set_use_rvo!(rvo_flag)
    set_avoid_staging_areas!(true)

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
        open(joinpath(results_path, prefix, "stats.toml"), "w") do io
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
