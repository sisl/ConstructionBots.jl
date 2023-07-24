"""
    run_lego_demo(;kwargs...)

Run the entire lego demo for model `project_name`.
"""
function run_lego_demo(;
    project_name                        ="X-wingFighter.mpd",
    MODEL_SCALE                         =0.004,
    NUM_ROBOTS                          =100,
    ROBOT_SCALE                         =MODEL_SCALE,
    ROBOT_HEIGHT                        =10 * ROBOT_SCALE,
    ROBOT_RADIUS                        =25 * ROBOT_SCALE,
    OBJECT_VTX_RANGE                    =(-10:10, -10:10, 0:1),
    HOME_VTX_RANGE                      =(-10:4*ROBOT_RADIUS:10, -10:4*ROBOT_RADIUS:10, 0:0),
    MAX_STEPS                           =8000,
    STAGING_BUFFER_FACTOR               =1.5,
    BUILD_STEP_BUFFER_FACTOR            =0.5,
    base_graphics_path                  =joinpath(dirname(pathof(ConstructionBots)), "..", "graphics"),
    graphics_path                       =joinpath(base_graphics_path, project_name),
    ASSIGNMENT_MODE                     =:GREEDY,
    visualize_processing                =false,
    visualize_animation_at_end          =false,
    save_animation                      =true,
    save_animation_along_the_way        =false,
    anim_steps                          =false,
    anim_active_areas                   =false,
    anim_interval                       =100,
    RVO_FLAG                            =true,
    OVERWRITE_RESULTS                   =false,
    WRITE_RESULTS                       =true,
    QUIT_AFTER_OPTIMAL                  =false,
    max_num_iters_no_progress           =Inf,
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
    # filename = joinpath(dirname(pathof(LDrawParser)), "..", "assets", project_name)
    filename = joinpath("/Users/dylan/Documents/Lego/LDrawParser.jl/assets", project_name)

    @assert ispath(filename) "File $(filename) does not exist."

    if RVO_FLAG == true
        prefix = "with_rvo"
    else
        prefix = "without_rvo"
    end
    if ASSIGNMENT_MODE == :OPTIMAL
        prefix = string("optimal_", prefix)
    else
        prefix = string("greedy_", prefix)
    end
    mkpath(joinpath(graphics_path, prefix))
    stats_path = joinpath(graphics_path, prefix, "stats.toml")
    if isfile(stats_path) && WRITE_RESULTS && !OVERWRITE_RESULTS
        @warn "Terminating because results are already compiled at $(stats_path)"
        return nothing
    end

    anim_path = joinpath(graphics_path, prefix, "construction_simulation.html")
    anim_prog_path = joinpath(graphics_path, prefix, "construction_simulation_")

    reset_all_id_counters!()
    reset_all_invalid_id_counters!()
    Random.seed!(seed)

    set_default_robot_geom!(
        Cylinder(Point(0.0, 0.0, 0.0), Point(0.0, 0.0, ROBOT_HEIGHT), ROBOT_RADIUS)
    )
    FactoryRendering.set_render_param!(:Radius, :Robot, ROBOT_RADIUS)

    PRE_EXECUTION_START_TIME = time()
    model = parse_ldraw_file(filename)
    populate_part_geometry!(model)
    LDrawParser.change_coordinate_system!(model, ldraw_base_transform(), MODEL_SCALE)

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
    START_GEOM_APPROX = time()
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HypersphereKey())
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HyperrectangleKey())
    GEOM_APPROX_TIME = time() - START_GEOM_APPROX
    print("done!\n")

    # Define TransportUnit configurations
    print("Configuring transport units...")
    CONFIG_TRANSPORT_UNITS_TIME = time()
    _, cvx_hulls = ConstructionBots.init_transport_units!(scene_tree;
        robot_radius=ROBOT_RADIUS
    )
    CONFIG_TRANSPORT_UNITS_TIME = time() - CONFIG_TRANSPORT_UNITS_TIME
    print("done!\n")

    # validate SceneTree
    print("Validating scene tree...")
    root = get_node(scene_tree, collect(get_all_root_nodes(scene_tree))[1])
    validate_tree(HierarchicalGeometry.get_transform_node(root))
    validate_embedded_tree(scene_tree, v -> HierarchicalGeometry.get_transform_node(get_node(scene_tree, v)))
    print("done!\n")

    ## Add some robots to scene tree
    vtxs = ConstructionBots.construct_vtx_array(; spacing=(1.0, 1.0, 0.0), ranges=(-10:10, -10:10, 0:0))
    robot_vtxs = draw_random_uniform(vtxs, NUM_ROBOTS)
    ConstructionBots.add_robots_to_scene!(scene_tree, robot_vtxs, [default_robot_geom()])

    # Add temporary dummy robots ############################
    ConstructionBots.add_temporary_invalid_robots!(scene_tree; with_edges=true)
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HypersphereKey())
    @assert all(map(node -> has_vertex(node.geom_hierarchy, HypersphereKey()), get_nodes(scene_tree)))
    HierarchicalGeometry.compute_approximate_geometries!(scene_tree, HyperrectangleKey())
    @assert all(map(node -> has_vertex(node.geom_hierarchy, HyperrectangleKey()), get_nodes(scene_tree)))
    # Remove temporary dummy robots ############################
    ConstructionBots.remove_temporary_invalid_robots!(scene_tree)

    ## Construct Partial Schedule
    print("Constructing partial schedule...")
    HierarchicalGeometry.jump_to_final_configuration!(scene_tree; set_edges=true)
    sched = construct_partial_construction_schedule(model, model_spec, scene_tree, id_map)
    @assert validate_schedule_transform_tree(sched)
    print("done!\n")

    ## Generate staging plan
    print("Generating staging plan...")
    MAX_OBJECT_TRANSPORT_UNIT_RADIUS = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)
    STAGING_PLAN_TIME = time()
    staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree, sched;
        buffer_radius=STAGING_BUFFER_FACTOR * MAX_OBJECT_TRANSPORT_UNIT_RADIUS,
        build_step_buffer_radius=BUILD_STEP_BUFFER_FACTOR * HierarchicalGeometry.default_robot_radius()
    )
    STAGING_PLAN_TIME = time() - STAGING_PLAN_TIME
    print("done!\n")

    if project_name == "X-wingFighter.mpd" || project_name == "X-wingMini.mpd"
        ac = get_node(sched, first(inneighbors(sched, ProjectComplete(1))))
        circ_center = HierarchicalGeometry.get_center(get_cached_geom(node_val(ac).outer_staging_circle))
        @assert has_parent(goal_config(ac), goal_config(ac))
        tform = relative_transform(
            CoordinateTransformations.Translation(circ_center...),
            global_transform(goal_config(ac)),
        )
        set_local_transform!(goal_config(ac), tform)
    end

    # record statistics
    STATS = Dict()
    STATS[:numobjects] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(ObjectNode, n)])
    STATS[:numassemblies] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(AssemblyNode, n)])
    STATS[:numrobots] = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(RobotNode, n)])
    STATS[:ConfigTransportUnitsTime] = CONFIG_TRANSPORT_UNITS_TIME
    STATS[:StagingPlanTime] = STAGING_PLAN_TIME
    if WRITE_RESULTS
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, STATS)
        end
    end


    # Move objects away from the staging plan
    print("Moving objects away from staging plan...")
    MAX_CARGO_HEIGHT = maximum(map(n -> get_base_geom(n, HyperrectangleKey()).radius[3] * 2,
        filter(n -> matches_template(TransportUnitNode, n), get_nodes(scene_tree))))
    vtxs = ConstructionBots.construct_vtx_array(;
        origin=SVector(0.0, 0.0, MAX_CARGO_HEIGHT),
        obstacles=collect(values(staging_circles)),
        ranges=OBJECT_VTX_RANGE
    )
    NUM_OBJECTS = length(filter(n -> matches_template(ObjectNode, n), get_nodes(scene_tree)))
    object_vtxs = draw_random_uniform(vtxs, NUM_OBJECTS)
    ConstructionBots.select_initial_object_grid_locations!(sched, object_vtxs)

    # Move assemblies up so they float above the robots
    for node in get_nodes(scene_tree)
        if matches_template(AssemblyNode, node)
            start_node = get_node(sched, AssemblyComplete(node))
            # raise start
            current = global_transform(start_config(start_node))
            rect = current(get_base_geom(node, HyperrectangleKey()))
            dh = MAX_CARGO_HEIGHT - (rect.center.-rect.radius)[3]
            # @show summary(node_id(node)), dh
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
    ConstructionBots.set_default_loading_speed!(10 * HierarchicalGeometry.default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(10 * HierarchicalGeometry.default_robot_radius())
    tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

    ASSIGNMENT_TIME = time()
    ## Black box MILP solver
    milp_model = SparseAdjacencyMILP()
    if ASSIGNMENT_MODE == :OPTIMAL
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
    ASSIGNMENT_TIME = time() - ASSIGNMENT_TIME
    POST_ASSIGNMENT_MAKESPAN = TaskGraphs.makespan(tg_sched)
    print("done!\n")

    # Try assigning robots to "home" locations so they don't sit around in each others' way
    go_nodes = [n for n in get_nodes(tg_sched) if matches_template(RobotGo, n) && is_terminal_node(tg_sched, n)]
    home_vtx_candidates = ConstructionBots.construct_vtx_array(;
        origin=SVector(0.0, 0.0, MAX_CARGO_HEIGHT),
        obstacles=collect(values(staging_circles)),
        ranges=HOME_VTX_RANGE
    )
    home_vtxs = draw_random_uniform(home_vtx_candidates, length(go_nodes))
    for (vtx, n) in zip(home_vtxs, go_nodes)
        HierarchicalGeometry.set_desired_global_transform!(goal_config(n),
            CoordinateTransformations.Translation(vtx[1], vtx[2], 0.0) ∘ identity_linear_map()
        )
    end

    # compile pre execution statistics
    PRE_EXECUTION_TIME = time() - PRE_EXECUTION_START_TIME
    if ASSIGNMENT_MODE == :OPTIMAL && isa(milp_model, AbstractGreedyAssignment)
        ASSIGNMENT_TIME = Inf
        POST_ASSIGNMENT_MAKESPAN = Inf
        PRE_EXECUTION_TIME = Inf
    end
    STATS[:AssigmentTime] = ASSIGNMENT_TIME
    STATS[:PreExecutionRuntime] = PRE_EXECUTION_TIME
    STATS[:OptimisticMakespan] = POST_ASSIGNMENT_MAKESPAN
    if WRITE_RESULTS
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, STATS)
        end
    end
    if ASSIGNMENT_MODE == :OPTIMAL && QUIT_AFTER_OPTIMAL
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

    # rvo
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
            dt_animate=0.0,
            dt=0.0,
            anim=anim,
            interp_steps=40
        )
        atframe(anim, current_frame(anim)) do
            set_scene_tree_to_initial_condition!(scene_tree, sched; remove_all_edges=true)
            update_visualizer!(factory_vis)
        end
        setanimation!(visualizer, anim.anim, play=false)
        print("done\n")

        if save_animation_along_the_way
            save_animation!(visualizer, "$(anim_prog_path)preprocessing.html")
        end

    end

    set_use_rvo!(RVO_FLAG)
    set_avoid_staging_areas!(true)

    EXECUTION_START_TIME = time()

    status, TIME_STEPS = simulate!(
        env,
        factory_vis,
        max_time_steps=MAX_STEPS,
        visualize_processing=visualize_processing,
        process_animation_tasks=process_animation_tasks,
        anim=anim,
        anim_steps=anim_steps,
        anim_active_areas=anim_active_areas,
        anim_interval=anim_interval,
        save_anim_prog_path=save_animation_along_the_way ? anim_prog_path : nothing,
        max_num_iters_no_progress=max_num_iters_no_progress
    )

    if status == true
        EXECUTION_TIME = time() - EXECUTION_START_TIME
    else
        EXECUTION_TIME = Inf
        TIME_STEPS = Inf
    end

    # Add results
    STATS[:ExecutionRuntime] = EXECUTION_TIME
    STATS[:Makespan] = TIME_STEPS * env.dt
    if WRITE_RESULTS
        open(joinpath(graphics_path, prefix, "stats.toml"), "w") do io
            TOML.print(io, STATS)
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
    return env, STATS
end

function save_animation!(visualizer, path)
    println("Saving animation to $(path)")
    open(path, "w") do io
        write(io, static_html(visualizer))
    end
end

function simulate!(
    env,
    factory_vis;
    max_time_steps=2000,
    visualize_processing=false,
    process_animation_tasks=false,
    anim=nothing,
    anim_interval=100,
    anim_steps=false,
    anim_active_areas=false,
    save_anim_prog_path=nothing,
    max_num_iters_no_progress=Inf
)
    @unpack sched, cache = env

    iters = 1
    ConstructionBots.step_environment!(env)
    newly_updated = ConstructionBots.update_planning_cache!(env, 0.0)

    step_1_closed = length(cache.closed_set)

    prog = nothing
    total_nodes = nv(sched)
    generate_showvalues(it, nc) = () -> [(:step_num,it), (:n_closed,nc), (:n_total,total_nodes)]
    if global_logger().min_level == Logging.LogLevel(2)
        prog = ProgressMeter.Progress(
            nv(sched) - step_1_closed;
            desc="Simulating...",
            barlen=50,
            showspeed=true,
            dt=1.0
        )
    end

    update_steps = []
    starting_step = 0
    if !isnothing(anim)
        starting_step = current_frame(anim)
    end

    last_iter_num_closed = length(cache.closed_set)
    num_iters_no_progress = 0
    for k in 2:max_time_steps
        iters = k

        ConstructionBots.step_environment!(env)
        newly_updated = ConstructionBots.update_planning_cache!(env, 0.0)

        if !isnothing(factory_vis.vis)
            if !isempty(newly_updated)
                scene_nodes, closed_steps_nodes, active_build_nodes, fac_active_flags_nodes = visualizer_update_function!(factory_vis, env, newly_updated)
                if visualize_processing
                    for node_i in closed_steps_nodes
                        setvisible!(node_i, false)
                    end
                    for node_i in active_build_nodes
                        setvisible!(node_i, true)
                    end
                    setvisible!(factory_vis.active_flags, false)
                    for node_key in fac_active_flags_nodes
                        setvisible!(factory_vis.active_flags[node_key], true)
                    end
                    update_visualizer!(factory_vis.vis_nodes, scene_nodes)
                    sleep(0.2) # Without this, we get a socket connection error
                end
                if process_animation_tasks
                    push!(update_steps,
                        (iters,
                        deepcopy(factory_vis.vis_nodes),
                        deepcopy(scene_nodes),
                        closed_steps_nodes,
                        active_build_nodes,
                        fac_active_flags_nodes)
                    )
                end
            end
        end

        if length(cache.closed_set) == last_iter_num_closed
            num_iters_no_progress += 1
        else
            num_iters_no_progress = 0
        end
        last_iter_num_closed = length(cache.closed_set)

        project_stop_bool = ConstructionBots.project_complete(env)
        if num_iters_no_progress >= max_num_iters_no_progress
            @warn "No progress for $num_iters_no_progress iterations. Terminating."
            project_stop_bool = true
        end



        if process_animation_tasks && (length(update_steps) >= anim_interval || project_stop_bool)
            for step_k in update_steps
                k_k = step_k[1]
                vis_nodes_k = step_k[2]
                scene_nodes_k = step_k[3]
                closed_steps_nodes_k = step_k[4]
                active_build_nodes_k = step_k[5]
                fac_active_flags_nodes_k = step_k[6]

                set_current_frame!(anim, k_k + starting_step)
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
            if !isnothing(save_anim_prog_path) && !project_stop_bool
                save_animation!(factory_vis.vis, "$(save_anim_prog_path)$(iters).html")
            end
        end

        if project_stop_bool
            ProgressMeter.finish!(prog)
            if ConstructionBots.project_complete(env)
                println("PROJECT COMPLETE!")
            else
                println("PROJECT INCOMPLETE!")
                var_dump_path = joinpath(dirname(pathof(ConstructionBots)), "..", "variable_dump")
                mkpath(var_dump_path)
                var_dump_path = joinpath(var_dump_path, "var_dump_$(iters).jld2")
                println("Dumping variables to $(var_dump_path)")
                var_dict = Dict(
                    "env" => env,
                    "factory_vis" => factory_vis,
                    "max_time_steps" => max_time_steps,
                    "visualize_processing" => visualize_processing,
                    "process_animation_tasks" => process_animation_tasks,
                    "anim_interval" => anim_interval,
                    "anim_steps" => anim_steps,
                    "anim_active_areas" => anim_active_areas,
                    "save_anim_prog_path" => save_anim_prog_path,
                    "max_num_iters_no_progress" => max_num_iters_no_progress
                )
                print("\tSaving...")
                JLD2.save(var_dump_path, var_dict)
                print("done!\n")
            end
            break
        end
        if !isnothing(prog)
            nc = length(cache.closed_set)
            ProgressMeter.update!(prog, nc - step_1_closed;
                showvalues=generate_showvalues(iters, nc)
            )
        end
    end
    return project_complete(env), iters
end
