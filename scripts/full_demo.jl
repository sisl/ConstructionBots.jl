using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using TaskGraphs
using JuMP

using Graphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations
using StaticArrays
using LinearAlgebra

using PyCall

using MeshCat
using Plots
using Random

using TOML
using Logging

using Colors
using Printf
using Parameters

using PGFPlots
using PGFPlotsX
using Measures
import Cairo
using Compose

using JLD2

using ProgressMeter

using StatsBase: std

using Gurobi

include("../deps/GraphPlottingBFS.jl")
include("../deps/FactoryRendering.jl")
include("../src/render_tools.jl")
include("../src/tg_render_tools.jl")

struct SimParameters
    sim_batch_size::Int
    max_time_steps::Int
    visualize_processing::Bool
    process_animation_tasks::Bool
    save_anim_interval::Int
    process_updates_interval::Int
    anim_steps::Bool
    anim_active_areas::Bool
    save_anim_prog_path::String
    max_num_iters_no_progress::Int
end

mutable struct SimProcessingData
    stop_simulating::Bool
    iter::Int
    starting_frame::Int
    prog::ProgressMeter.Progress
    num_closed_step_1::Int
    last_iter_num_closed::Int
    num_iters_no_progress::Int
    num_iters_since_anim_save::Int
    progress_update_fcn::Function
end

"""
    run_lego_demo(;kwargs...)

Run the entire lego demo for model `project_name`.
"""
function run_lego_demo(;
    project_name                        ="tractor.mpd",
    model_scale                         =0.008,
    num_robots                          =12,
    robot_scale                         =model_scale * 0.7,
    robot_height                        =10 * robot_scale,
    robot_radius                        =25 * robot_scale,
    num_object_layers                   =1,
    max_steps                           =100000,
    staging_buffer_factor               =1.2,
    build_step_buffer_factor            =0.5,
    base_graphics_path                  =joinpath(dirname(pathof(ConstructionBots)), "..", "graphics"),
    graphics_path                       =joinpath(base_graphics_path, project_name),
    assignment_mode                     =:GREEDY,
    visualize_processing                =false,
    visualize_animation_at_end          =false,
    save_animation                      =true,
    save_animation_along_the_way        =false,
    anim_steps                          =false,
    anim_active_areas                   =false,
    process_updates_interval            =25,
    save_anim_interval                  =500,
    rvo_flag                            =true,
    overwrite_results                   =false,
    write_results                       =true,
    quit_after_optimal                  =false,
    max_num_iters_no_progress           =Inf,
    sim_batch_size                      =50,
    log_level                           =Logging.LogLevel(2),
    default_milp_optimizer              =Gurobi.Optimizer, #! Need to change to non-Gurobi
    optimizer_time_limit                =100,
    seed                                =1
)

    process_animation_tasks = save_animation || save_animation_along_the_way || visualize_animation_at_end
    @assert !(process_animation_tasks && visualize_processing) "Cannot visualize processing while processing animation tasks."

    if save_animation_along_the_way
        save_animation = true
    end

    visualizer = nothing
    if process_animation_tasks || visualize_processing
        visualizer = MeshCat.Visualizer()
    end

    mkpath(graphics_path)
    filename = joinpath(dirname(pathof(ConstructionBots)), "..", "LDraw_files", project_name)
    @assert ispath(filename) "File $(filename) does not exist."

    global_logger(ConsoleLogger(stderr, log_level))
    set_default_milp_optimizer!(default_milp_optimizer)
    TaskGraphs.set_default_optimizer_attributes!(
        "TimeLimit"=>optimizer_time_limit,
        MOI.Silent()=>false
    )

    if rvo_flag == true
        prefix = "with_rvo"
    else
        prefix = "without_rvo"
    end
    if assignment_mode == :OPTIMAL
        prefix = string("optimal_", prefix)
    else
        prefix = string("greedy_", prefix)
    end
    mkpath(joinpath(graphics_path, prefix))
    stats_path = joinpath(graphics_path, prefix, "stats.toml")
    if isfile(stats_path) && write_results && !overwrite_results
        @warn "Terminating because results are already compiled at $(stats_path)"
        return nothing
    end

    anim_path = joinpath(graphics_path, prefix, "construction_simulation.html")
    anim_prog_path = joinpath(graphics_path, prefix, "construction_simulation_")


    sim_params = SimParameters(
        sim_batch_size,
        max_steps,
        visualize_processing,
        process_animation_tasks,
        save_anim_interval,
        process_updates_interval,
        anim_steps,
        anim_active_areas,
        anim_prog_path,
        max_num_iters_no_progress
    )

    reset_all_id_counters!()
    reset_all_invalid_id_counters!()
    Random.seed!(seed)

    set_default_robot_geom!(
        Cylinder(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, robot_height), robot_radius)
    )
    FactoryRendering.set_render_param!(:Radius, :Robot, robot_radius)

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
    robot_vtxs = draw_random_uniform(vtxs, num_robots)
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
    stats = Dict()
    stats[:numobjects] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(ObjectNode, n)])
    stats[:numassemblies] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(AssemblyNode, n)])
    stats[:numrobots] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(RobotNode, n)])
    stats[:ConfigTransportUnitsTime] = config_transport_units_time
    stats[:StagingPlanTime] = staging_plan_time
    if write_results
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
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

    #? Can we use the greedy assignment to seed the MILP?!
    assignment_time = time()
    ## Black box MILP solver
    milp_model = SparseAdjacencyMILP()
    if assignment_mode == :OPTIMAL
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        optimize!(milp_model)
    end
    if primal_status(milp_model) == MOI.NO_SOLUTION
        ## Greedy Assignment with enforced build-step ordering
        milp_model = ConstructionBots.GreedyOrderedAssignment(
            greedy_cost=TaskGraphs.GreedyFinalTimeCost(),
        )
        milp_model = formulate_milp(milp_model, tg_sched, scene_tree)
        optimize!(milp_model)
    end
    validate_schedule_transform_tree(
        ConstructionBots.convert_from_operating_schedule(typeof(sched), tg_sched)
        ; post_staging=true)
    update_project_schedule!(nothing, milp_model, tg_sched, scene_tree)
    @assert validate(tg_sched)
    assignment_time = time() - assignment_time
    post_assignment_makespan = TaskGraphs.makespan(tg_sched)
    print("done!\n")

    # Assign robots to "home" locations so they don't sit around in each others' way
    go_nodes = [n for n in get_nodes(tg_sched) if matches_template(RobotGo, n) && is_terminal_node(tg_sched, n)]
    min_assembly_1_xy = (staging_circles[AssemblyID(1)].center .- staging_circles[AssemblyID(1)].radius)[1:2]
    # home_vtxs = draw_random_uniform(home_vtx_candidates, length(go_nodes))
    for (ii, n) in enumerate(go_nodes)
        vtx_x = min_assembly_1_xy[1] - ii * 5 * robot_radius
        vtx_y = min_assembly_1_xy[2]
        HierarchicalGeometry.set_desired_global_transform!(goal_config(n),
            CoordinateTransformations.Translation(vtx_x, vtx_y, 0.0) ∘ identity_linear_map()
        )
    end

    # compile pre execution statistics
    pre_execution_time = time() - pre_execution_start_time
    if assignment_mode == :OPTIMAL && isa(milp_model, AbstractGreedyAssignment)
        assignment_time = Inf
        post_assignment_makespan = Inf
        pre_execution_time = Inf
    end
    stats[:AssigmentTime] = assignment_time
    stats[:PreExecutionRuntime] = pre_execution_time
    stats[:OptimisticMakespan] = post_assignment_makespan
    if write_results
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, stats)
        end
    end
    if assignment_mode == :OPTIMAL && quit_after_optimal
        return nothing
    end


    if !isnothing(visualizer)
        if visualize_processing
            println("Visualizer started at $(visualizer.core.host):$(visualizer.core.port)")
            open(visualizer)
        end
        delete!(visualizer)
    end

    factory_vis = FactoryVisualizer(vis=visualizer)
    if !isnothing(visualizer)
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

    #? Can we only set the RVO module if we are using it? Might avoid the requirement of
    #? having to install the RVO module if we don't want to use it (make it easier for
    #? others to run the code just focusing on the scheduling)
    # rvo
    ConstructionBots.reset_rvo_python_module!()

    ConstructionBots.set_rvo_default_time_step!(1 / 40.0)
    ConstructionBots.set_rvo_default_neighbor_distance!(16 * HierarchicalGeometry.default_robot_radius()) # 4
    ConstructionBots.set_rvo_default_min_neighbor_distance!(10 * HierarchicalGeometry.default_robot_radius()) # 3
    ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(; horizon=2.0))
    ConstructionBots.set_staging_buffer_radius!(HierarchicalGeometry.default_robot_radius())

    env = PlannerEnv(
        sched=tg_sched,
        scene_tree=scene_tree,
        staging_circles=staging_circles
    )
    active_nodes = (get_node(tg_sched, v) for v in env.cache.active_set)
    ConstructionBots.rvo_add_agents!(scene_tree, active_nodes)

    static_potential_function = (x, r) -> 0.0
    pairwise_potential_function = ConstructionBots.repulsion_potential

    #? Should we add an option here to allow for processing without the tangent bug or
    #? potential field controllers?
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = HierarchicalGeometry.get_radius(get_base_geom(n, HypersphereKey()))
            vmax = ConstructionBots.get_rvo_max_speed(n)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy=TangentBugPolicy(
                    dt=env.dt,
                    vmax=vmax,
                    agent_radius=agent_radius,
                ),
                dispersion_policy=ConstructionBots.PotentialFieldController(
                    agent=n,
                    node=node,
                    agent_radius=agent_radius,
                    vmax=vmax,
                    max_buffer_radius=2.5 * HierarchicalGeometry.default_robot_radius(),
                    static_potentials=static_potential_function,
                    pairwise_potentials=pairwise_potential_function,
                )
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
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, stats)
        end
    end

    if process_animation_tasks
        if save_animation
            save_animation!(visualizer, anim_path)
        end
        if visualize_animation_at_end
            open(visualizer)
        end
    end
    return env, stats
end
"""
    run_simulation!(env, factory_vis, anim, sim_params)

Run a simulation of the environment. Currently, this wraps another simulation function to
help with memory issues. The goal is to refactor the code to not require this "hack".
"""
function run_simulation!(
    env::PlannerEnv,
    factory_vis::FactoryVisualizer,
    anim::Union{AnimationWrapper,Nothing},
    sim_params::SimParameters
)

    it = 1
    ConstructionBots.step_environment!(env)
    ConstructionBots.update_planning_cache!(env, 0.0)

    step_1_closed = length(env.cache.closed_set)
    total_nodes = nv(env.sched)
    generate_showvalues(it, nc) = () -> [(:step_num,it), (:n_closed,nc), (:n_total,total_nodes)]
    prog = ProgressMeter.Progress(
        nv(env.sched) - step_1_closed;
        desc="Simulating...",
        barlen=50,
        showspeed=true,
        dt=1.0
    )

    starting_frame = 0
    if !isnothing(anim)
        starting_frame = current_frame(anim)
    end

    sim_process_data = SimProcessingData(
        false, 1, starting_frame, prog, step_1_closed, step_1_closed, 0, 0, generate_showvalues
    )

    up_steps = []
    while !sim_process_data.stop_simulating && sim_process_data.iter < max_steps
        up_steps = simulate!(env, factory_vis, anim, sim_params, sim_process_data, up_steps)
    end

    return ConstructionBots.project_complete(env), sim_process_data.iter
end

function simulate!(
    env::PlannerEnv,
    factory_vis::FactoryVisualizer,
    anim::Union{AnimationWrapper,Nothing},
    sim_params::SimParameters,
    sim_process_data::SimProcessingData,
    update_steps::Vector
)

    @unpack sched, cache = env
    @unpack sim_batch_size, max_time_steps, visualize_processing = sim_params
    @unpack process_animation_tasks, save_anim_interval, process_updates_interval = sim_params
    @unpack anim_steps, anim_active_areas, save_anim_prog_path, max_num_iters_no_progress = sim_params

    for _ in 1:sim_batch_size
        sim_process_data.iter += 1

        ConstructionBots.step_environment!(env)
        newly_updated = ConstructionBots.update_planning_cache!(env, 0.0)

        if !isnothing(factory_vis.vis)
            if !isempty(newly_updated)
                scene_nodes, closed_steps_nodes, active_build_nodes, fac_active_flags_nodes = visualizer_update_function!(factory_vis, env, newly_updated)
                sim_process_data.num_iters_since_anim_save += 1
                if process_animation_tasks
                    update_tuple = (sim_process_data.iter, deepcopy(factory_vis.vis_nodes),
                        deepcopy(scene_nodes), closed_steps_nodes, active_build_nodes,
                        fac_active_flags_nodes)
                    push!(update_steps, update_tuple)
                end
            end
        end

        if length(cache.closed_set) == sim_process_data.last_iter_num_closed
            sim_process_data.num_iters_no_progress += 1
        else
            sim_process_data.num_iters_no_progress = 0
        end
        sim_process_data.last_iter_num_closed = length(cache.closed_set)

        project_stop_bool = ConstructionBots.project_complete(env)
        if sim_process_data.num_iters_no_progress >= max_num_iters_no_progress
            @warn "No progress for $(sim_process_data.num_iters_no_progress) iterations. Terminating."
            project_stop_bool = true
        end

        if process_animation_tasks &&
            (length(update_steps) >= process_updates_interval || project_stop_bool)

            for step_k in update_steps
                k_k = step_k[1]
                vis_nodes_k = step_k[2]
                scene_nodes_k = step_k[3]
                closed_steps_nodes_k = step_k[4]
                active_build_nodes_k = step_k[5]
                fac_active_flags_nodes_k = step_k[6]

                set_current_frame!(anim, k_k + sim_process_data.starting_frame)
                atframe(anim, current_frame(anim)) do
                    if anim_steps
                        for node_i in closed_steps_nodes_k
                            setvisible!(node_i, false)
                        end
                        for node_i in active_build_nodes_k
                            setvisible!(node_i, true)
                        end
                    end
                    if anim_active_areas
                        setvisible!(factory_vis.active_flags, false)
                        for node_key in fac_active_flags_nodes_k
                            setvisible!(factory_vis.active_flags[node_key], true)
                        end
                    end
                    update_visualizer!(vis_nodes_k, scene_nodes_k)
                end
            end
            setanimation!(factory_vis.vis, anim.anim, play=false)
            update_steps = []

            if (!isnothing(save_anim_prog_path) &&
                !project_stop_bool &&
                sim_process_data.num_iters_since_anim_save >= save_anim_interval)

                save_animation!(factory_vis.vis, "$(save_anim_prog_path)$(sim_process_data.iter).html")
                sim_process_data.num_iters_since_anim_save = 0
            end
        end

        nc = length(cache.closed_set)
        ProgressMeter.update!(
            sim_process_data.prog, nc - sim_process_data.num_closed_step_1;
            showvalues=sim_process_data.progress_update_fcn(sim_process_data.iter, nc)
        )

        if project_stop_bool
            ProgressMeter.finish!(sim_process_data.prog)
            if ConstructionBots.project_complete(env)
                println("PROJECT COMPLETE!")
            else
                println("PROJECT INCOMPLETE!")
                var_dump_path = joinpath(dirname(pathof(ConstructionBots)), "..", "variable_dump")
                mkpath(var_dump_path)
                var_dump_path = joinpath(var_dump_path, "var_dump_$(sim_process_data.iter).jld2")
                println("Dumping variables to $(var_dump_path)")
                var_dict = Dict(
                    "env" => env,
                    "factory_vis" => factory_vis,
                    "anim" => anim,
                    "sim_params" => sim_params,
                    "sim_process_data" => sim_process_data
                )
                print("\tSaving...")
                JLD2.save(var_dump_path, var_dict)
                print("done!\n")
            end
        end

        sim_process_data.stop_simulating = project_stop_bool
        if sim_process_data.stop_simulating
            break
        end
    end

    return update_steps
end

function save_animation!(visualizer, path)
    println("Saving animation to $(path)")
    open(path, "w") do io
        write(io, static_html(visualizer))
    end
end

function get_buildstep_circles(sched)
    circles = Dict{AbstractID, Ball2}()
    for n in node_iterator(sched, topological_sort_by_dfs(sched))
        if matches_template(OpenBuildStep, n)
            id = node_id(n)
            sphere = get_cached_geom(node_val(n).staging_circle)
            ctr = HierarchicalGeometry.project_to_2d(sphere.center)
            circles[id] = Ball2(ctr, sphere.radius)
        end
    end
    return circles
end

function shift_staging_circles(staging_circles::Dict{AbstractID, Ball2}, sched, scene_tree)
    shifted_circles = Dict{AbstractID, Ball2}()
    for (id, geom) in staging_circles
        node = get_node(scene_tree, id)
        cargo = get_node(scene_tree, id)
        if isa(cargo, AssemblyNode)
            cargo_node = get_node(sched, AssemblyComplete(cargo))
        else
            cargo_node = get_node!(sched, ObjectStart(cargo, TransformNode()))
        end

        translate = HierarchicalGeometry.project_to_2d(global_transform(start_config(cargo_node)).translation)
        tform = CoordinateTransformations.Translation(translate)
        new_center = tform(geom.center)
        shifted_circles[id] = Ball2(new_center, geom.radius)

    end
    return shifted_circles
end

function remove_redundant(balls::Vector{Ball2}; ϵ=1e-4)
    redundant = falses(length(balls))
    for ii in 1:length(balls)
        for jj in 1:length(balls)
            if ii != jj && !redundant[jj] && contained_in(balls[ii], balls[jj]; ϵ=ϵ)
                redundant[ii] = true
            end
        end
    end
    return balls[.!redundant]
end

function contained_in(test_ball::Ball2, outer_ball::Ball2; ϵ=1e-4)
    return norm(test_ball.center - outer_ball.center) + test_ball.radius <= outer_ball.radius + ϵ
end

function get_object_vtx(scene_tree, obstacles, max_cargo_height, num_layers, robot_d)
    # Use object height to stack objects on top of each other as needed
    objects_hyperrects = map(n -> get_base_geom(n, HyperrectangleKey()),
        filter(n -> matches_template(ObjectNode, n), get_nodes(scene_tree)))

    max_object_x = maximum([ob.radius[1] * 2 for ob in objects_hyperrects])
    max_object_y = maximum([ob.radius[2] * 2 for ob in objects_hyperrects])
    max_object_h = maximum([ob.radius[3] * 2 for ob in objects_hyperrects])

    mean_object_x = sum([ob.radius[1] * 2 for ob in objects_hyperrects]) / length(objects_hyperrects)
    mean_object_y = sum([ob.radius[2] * 2 for ob in objects_hyperrects]) / length(objects_hyperrects)

    std_object_x = std([ob.radius[1] * 2 for ob in objects_hyperrects])
    std_object_y = std([ob.radius[2] * 2 for ob in objects_hyperrects])

    num_objects = length(filter(n -> matches_template(ObjectNode, n), get_nodes(scene_tree)))

    object_x_delta = min(max_object_x, mean_object_x + 3 * std_object_x) + robot_d * 2
    object_y_delta = min(max_object_y, mean_object_y + 3 * std_object_y) + robot_d * 2

    inflate_delta = max(max_object_x, max_object_y)

    inflated_obstacles = []
    for ob in obstacles
        push!(inflated_obstacles, Ball2(ob.center, ob.radius + inflate_delta))
    end

    num_objs_per_layer = ceil(num_objects / num_layers)

    init_width = sqrt(num_objs_per_layer * object_x_delta * object_y_delta)/2

    x_range = -init_width:object_x_delta:init_width
    y_range = -init_width:object_y_delta:init_width
    sweep_ranges = (x_range, y_range, 0:0)

    found_size = false
    cnt = 0
    while !found_size
        pts = ConstructionBots.construct_vtx_array(;
            origin=SVector{3, Float64}(0.0, 0.0, 0.0),
            obstacles=inflated_obstacles,
            ranges=sweep_ranges
        )
        if length(pts) * num_layers >= num_objects
            found_size = true
        else
            cnt += 1
            x_range = (-init_width - cnt * object_x_delta):object_x_delta:(init_width + cnt * object_x_delta)
            y_range = (-init_width - cnt * object_y_delta):object_y_delta:(init_width + cnt * object_y_delta)
            sweep_ranges = (x_range, y_range, 0:0)
        end
    end

    object_vtx_range = (
        x_range,
        y_range,
        0:num_layers-1
    )

    vtxs = ConstructionBots.construct_vtx_array(;
        origin=SVector{3, Float64}(0.0, 0.0, max_cargo_height),
        obstacles=inflated_obstacles,
        ranges=object_vtx_range,
        spacing=(1.0, 1.0, max_object_h)
    )

    return vtxs
end
