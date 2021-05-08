"""
    run_lego_demo(;kwargs...)

Run the entire lego demo for model `project_name`.
"""
function run_lego_demo(;
        project_name = "X-wingFighter.mpd",
        MODEL_SCALE         = 0.004,
        NUM_ROBOTS          = 100,
        ROBOT_SCALE         = MODEL_SCALE,
        ROBOT_HEIGHT        = 10*ROBOT_SCALE,
        ROBOT_RADIUS        = 25*ROBOT_SCALE,
        OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1),
        HOME_VTX_RANGE      = (-10:4*ROBOT_RADIUS:10,-10:4*ROBOT_RADIUS:10,0:0),
        MAX_STEPS           = 8000,
        STAGING_BUFFER_FACTOR = 1.5,
        BUILD_STEP_BUFFER_FACTOR = 0.5,
        base_graphics_path = "/scratch/Repositories/Sandbox/thesis_graphics/LEGO",
        graphics_path = joinpath(base_graphics_path,project_name),
        ASSIGNMENT_MODE     = :GREEDY,
        VISUALIZER          = true,
        vis                 = VISUALIZER ? MeshCat.Visualizer() : nothing,
        anim                = VISUALIZER ? AnimationWrapper(0) : nothing,
        RVO_FLAG            = true,
        OVERWRITE_RESULTS   = false,
        WRITE_RESULTS       = true,
        QUIT_AFTER_OPTIMAL  = true,
    )
    mkpath(graphics_path)
    filename = joinpath(dirname(pathof(LDrawParser)),"..","assets",project_name)

    if RVO_FLAG == true
        prefix = "with_rvo"
    else
        prefix = "without_rvo"
    end
    if ASSIGNMENT_MODE == :OPTIMAL
        prefix = string("optimal_",prefix)
    else
        prefix = string("greedy_",prefix)
    end
    mkpath(joinpath(graphics_path,prefix))
    stats_path = joinpath(graphics_path,prefix,"stats.toml")
    if isfile(stats_path) && WRITE_RESULTS && !OVERWRITE_RESULTS
        @warn "Terminating because results are already compiled at $(stats_path)"
        return nothing
    end

    if !(vis === nothing)
        delete!(vis)
    end

    reset_all_id_counters!()
    reset_all_invalid_id_counters!()
    Random.seed!(0);

    set_default_robot_geom!(
        Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,ROBOT_HEIGHT), ROBOT_RADIUS)
    )
    FR.set_render_param!(:Radius,:Robot,ROBOT_RADIUS)

    PRE_EXECUTION_START_TIME = time()
    model = parse_ldraw_file(filename)
    populate_part_geometry!(model);
    LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),MODEL_SCALE);

    ## CONSTRUCT MODEL SPEC
    spec = ConstructionBots.construct_model_spec(model)
    model_spec = ConstructionBots.extract_single_model(spec)
    id_map = ConstructionBots.build_id_map(model,model_spec)
    color_map = construct_color_map(model_spec,id_map)
    @assert GraphUtils.validate_graph(model_spec)

    ## CONSTRUCT SceneTree
    assembly_tree = ConstructionBots.construct_assembly_tree(model,model_spec,id_map)
    scene_tree = ConstructionBots.convert_to_scene_tree(assembly_tree)
    print(scene_tree,v->"$(summary(node_id(v))) : $(get(id_map,node_id(v),nothing))","\t")

    # Compute Approximate Geometry 
    START_GEOM_APPROX = time()
    HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
    HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
    GEOM_APPROX_TIME = time() - START_GEOM_APPROX

    # Define TransportUnit configurations
    CONFIG_TRANSPORT_UNITS_TIME = time()
    _, cvx_hulls = ConstructionBots.init_transport_units!(scene_tree;
        robot_radius = ROBOT_RADIUS,
        )
    CONFIG_TRANSPORT_UNITS_TIME = time() - CONFIG_TRANSPORT_UNITS_TIME

    # validate SceneTree
    root = get_node(scene_tree,collect(get_all_root_nodes(scene_tree))[1])
    validate_tree(HierarchicalGeometry.get_transform_node(root))
    validate_embedded_tree(scene_tree,v->HierarchicalGeometry.get_transform_node(get_node(scene_tree,v)))

    ## Add some robots to scene tree
    vtxs = ConstructionBots.construct_vtx_array(;spacing=(1.0,1.0,0.0), ranges=(-10:10,-10:10,0:0))
    robot_vtxs = draw_random_uniform(vtxs,NUM_ROBOTS)
    ConstructionBots.add_robots_to_scene!(scene_tree,robot_vtxs,[default_robot_geom()])

    # Add temporary dummy robots ############################
    ConstructionBots.add_temporary_invalid_robots!(scene_tree;with_edges=true)
    HG.compute_approximate_geometries!(scene_tree,HypersphereKey())
    @assert all(map(node->has_vertex(node.geom_hierarchy,HypersphereKey()), get_nodes(scene_tree)))
    HG.compute_approximate_geometries!(scene_tree,HyperrectangleKey())
    @assert all(map(node->has_vertex(node.geom_hierarchy,HyperrectangleKey()), get_nodes(scene_tree)))
    # Remove temporary dummy robots ############################
    ConstructionBots.remove_temporary_invalid_robots!(scene_tree)

    ## Construct Partial Schedule
    HG.jump_to_final_configuration!(scene_tree;set_edges=true)
    sched = construct_partial_construction_schedule(model,model_spec,scene_tree,id_map)
    @assert validate_schedule_transform_tree(sched)

    ## Generata staging plan
    MAX_OBJECT_TRANSPORT_UNIT_RADIUS = ConstructionBots.get_max_object_transport_unit_radius(scene_tree)
    STAGING_PLAN_TIME = time()
    staging_circles, bounding_circles = ConstructionBots.generate_staging_plan!(scene_tree,sched;
        buffer_radius=STAGING_BUFFER_FACTOR*MAX_OBJECT_TRANSPORT_UNIT_RADIUS,
        build_step_buffer_radius=BUILD_STEP_BUFFER_FACTOR*HG.default_robot_radius(),
    );
    STAGING_PLAN_TIME = time() - STAGING_PLAN_TIME

    # record statistics
    STATS = Dict()
    STATS[:numobjects]          = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(ObjectNode,n)])
    STATS[:numassemblies]       = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(AssemblyNode,n)])
    STATS[:numrobots]           = length([node_id(n) for n in get_nodes(scene_tree) if matches_template(RobotNode,n)])
    STATS[:ConfigTransportUnitsTime] = CONFIG_TRANSPORT_UNITS_TIME 
    STATS[:StagingPlanTime]     = STAGING_PLAN_TIME
    if WRITE_RESULTS
        open(joinpath(graphics_path,prefix,"stats.toml"),"w") do io
            TOML.print(io,STATS)
        end
    end


    # Move objects away from the staging plan
    MAX_CARGO_HEIGHT = maximum(map(n->get_base_geom(n,HyperrectangleKey()).radius[3]*2,
        filter(n->matches_template(TransportUnitNode,n),get_nodes(scene_tree))))
    vtxs = ConstructionBots.construct_vtx_array(;
        origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
        obstacles=collect(values(staging_circles)),
        ranges=OBJECT_VTX_RANGE,
        )
    NUM_OBJECTS = length(filter(n->matches_template(ObjectNode,n),get_nodes(scene_tree)))
    object_vtxs = draw_random_uniform(vtxs,NUM_OBJECTS)
    ConstructionBots.select_initial_object_grid_locations!(sched,object_vtxs)

    # Move assemblies up so they float above the robots
    for node in get_nodes(scene_tree)
        if matches_template(AssemblyNode,node) 
            start_node = get_node(sched,AssemblyComplete(node))
            # raise start 
            current = global_transform(start_config(start_node))
            rect = current(get_base_geom(node,HyperrectangleKey()))
            dh = MAX_CARGO_HEIGHT - (rect.center .- rect.radius)[3]
            # @show summary(node_id(node)), dh
            set_desired_global_transform_without_affecting_children!(
                start_config(start_node),
                CT.Translation(current.translation[1:2]...,dh) ∘ CT.LinearMap(current.linear)
            )
        end
    end
    # Make sure all transforms line up
    ConstructionBots.calibrate_transport_tasks!(sched)
    @assert validate_schedule_transform_tree(sched;post_staging=true)

    # Task Assignments
    ConstructionBots.add_dummy_robot_go_nodes!(sched)
    @assert validate_schedule_transform_tree(sched;post_staging=true)

    # Convert to OperatingSchedule
    ConstructionBots.set_default_loading_speed!(10*HG.default_robot_radius())
    ConstructionBots.set_default_rotational_loading_speed!(10*HG.default_robot_radius())
    tg_sched = ConstructionBots.convert_to_operating_schedule(sched)

    ASSIGNMENT_TIME = time()
    ## Black box MILP solver
    milp_model = SparseAdjacencyMILP()
    if ASSIGNMENT_MODE == :OPTIMAL
        milp_model = formulate_milp(milp_model,tg_sched,scene_tree)
        optimize!(milp_model)
    end
    if primal_status(milp_model) == MOI.NO_SOLUTION
        ## Greedy Assignment with enforced build-step ordering
        milp_model = ConstructionBots.GreedyOrderedAssignment(
            greedy_cost = TaskGraphs.GreedyFinalTimeCost(),
        )
        milp_model = formulate_milp(milp_model,tg_sched,scene_tree)
        optimize!(milp_model)
    end
    validate_schedule_transform_tree(
        ConstructionBots.convert_from_operating_schedule(typeof(sched),tg_sched)
        ;post_staging=true)
    update_project_schedule!(nothing,milp_model,tg_sched,scene_tree)
    @assert validate(tg_sched)
    ASSIGNMENT_TIME = time() - ASSIGNMENT_TIME
    POST_ASSIGNMENT_MAKESPAN = TaskGraphs.makespan(tg_sched)

    # Try assigning robots to "home" locations so they don't sit around in each others' way
    go_nodes = [n for n in get_nodes(tg_sched) if matches_template(RobotGo,n) && is_terminal_node(tg_sched,n)]
    home_vtx_candidates = ConstructionBots.construct_vtx_array(;
        origin=SVector(0.0,0.0,MAX_CARGO_HEIGHT),
        obstacles=collect(values(staging_circles)),
        ranges=HOME_VTX_RANGE,
        )
    home_vtxs = draw_random_uniform(home_vtx_candidates,length(go_nodes))
    for (vtx,n) in zip(home_vtxs,go_nodes)
        HG.set_desired_global_transform!(goal_config(n),
            CT.Translation(vtx[1],vtx[2],0.0) ∘ identity_linear_map()
        )
    end

    # compile pre execution statistics
    PRE_EXECUTION_TIME = time() - PRE_EXECUTION_START_TIME
    if ASSIGNMENT_MODE == :OPTIMAL && isa(milp_model,AbstractGreedyAssignment)
        ASSIGNMENT_TIME = Inf
        POST_ASSIGNMENT_MAKESPAN = Inf
        PRE_EXECUTION_TIME = Inf
    end
    STATS[:AssigmentTime]       = ASSIGNMENT_TIME
    STATS[:PreExecutionRuntime] = PRE_EXECUTION_TIME
    STATS[:OptimisticMakespan]  = POST_ASSIGNMENT_MAKESPAN
    if WRITE_RESULTS
        open(joinpath(graphics_path,prefix,"stats.toml"),"w") do io
            TOML.print(io,STATS)
        end
    end
    if ASSIGNMENT_MODE == :OPTIMAL && QUIT_AFTER_OPTIMAL
        return nothing
    end

    vis_nodes = Dict()
    staging_nodes = Dict()
    base_geom_nodes = Dict()
    if VISUALIZER
        ## Visualize assembly
        delete!(vis)
        vis_nodes, base_geom_nodes = populate_visualizer!(scene_tree,vis;
            color_map=color_map,
            color=RGB(0.3,0.3,0.3),
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
        setvisible!(sphere_nodes,false)
        setvisible!(rect_nodes,false)
        setvisible!(staging_nodes,false)

        # # restore correct configuration
        HG.jump_to_final_configuration!(scene_tree;set_edges=true)
        # update_visualizer!(scene_tree,vis_nodes)
        # # set staging plan and visualize
        set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
        # update_visualizer!(scene_tree,vis_nodes)

        # render video!
        anim = AnimationWrapper(0)
        # anim = nothing
        atframe(anim,current_frame(anim)) do
            HG.jump_to_final_configuration!(scene_tree;set_edges=true)
            update_visualizer!(scene_tree,vis_nodes)
            # setvisible!(sphere_nodes,false)
            setvisible!(vis_nodes,true)
            setvisible!(rect_nodes,false)
            setvisible!(staging_nodes,false)
        end
        step_animation!(anim)
        animate_preprocessing_steps!(
                vis,
                vis_nodes,
                scene_tree,
                sched,
                rect_nodes,
                base_geom_nodes,
                ;
                dt_animate=0.0,
                dt=0.0,
                anim=anim,
                interp_steps=40,
            )
        atframe(anim,current_frame(anim)) do
            set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)
            update_visualizer!(scene_tree,vis_nodes)
        end
        setanimation!(vis,anim.anim)
        if WRITE_RESULTS
            open(joinpath(graphics_path,"animate_preprocessing.html"),"w") do io
                write(io,static_html(vis))
            end
        end
    end

    set_scene_tree_to_initial_condition!(scene_tree,sched;remove_all_edges=true)

    # rvo
    ConstructionBots.set_rvo_default_time_step!(1/40.0)
    ConstructionBots.set_rvo_default_neighbor_distance!(16*HG.default_robot_radius()) # 4
    ConstructionBots.set_rvo_default_min_neighbor_distance!(10*HG.default_robot_radius()) # 3
    ConstructionBots.rvo_set_new_sim!(ConstructionBots.rvo_new_sim(;horizon=2.0))
    ConstructionBots.set_staging_buffer_radius!(HG.default_robot_radius())
    env = PlannerEnv(
            sched=tg_sched,
            scene_tree=scene_tree,
            staging_circles=staging_circles
            )
    active_nodes = (get_node(tg_sched,v) for v in env.cache.active_set)
    ConstructionBots.rvo_add_agents!(scene_tree,active_nodes)

    for n in get_nodes(scene_tree)
        if matches_template(Union{RobotNode,TransportUnitNode},n)
            env.agent_policies[node_id(n)] = TangentBugPolicy(
                dt = env.dt,
                vmax = ConstructionBots.get_rvo_max_speed(n),
                agent_radius = HG.get_radius(get_base_geom(n,HypersphereKey())),
            )
        end
    end

    update_visualizer_function = construct_visualizer_update_function(vis,vis_nodes,staging_nodes;
        anim=anim,
        )

    set_use_rvo!(RVO_FLAG)
    set_avoid_staging_areas!(true)

    EXECUTION_START_TIME = time()
    status, TIME_STEPS = ConstructionBots.simulate!(env, update_visualizer_function,
        max_time_steps=MAX_STEPS,
        )
    if status == true
        EXECUTION_TIME = time() -  EXECUTION_START_TIME
    else
        EXECUTION_TIME = Inf
        TIME_STEPS = Inf
    end
    
    # Add results
    STATS[:ExecutionRuntime]    = EXECUTION_TIME
    STATS[:Makespan]            = TIME_STEPS*env.dt
    if WRITE_RESULTS
        open(joinpath(graphics_path,prefix,"stats.toml"),"w") do io
            TOML.print(io,STATS)
        end
    end

    if VISUALIZER
        setanimation!(vis,anim.anim)
        if WRITE_RESULTS
            open(joinpath(graphics_path,prefix,"construction_simulation.html"),"w") do io
                write(io,static_html(vis))
            end
        end
        render(vis)

        # animate camera path
        # rotate_camera!(vis,anim);
        # setanimation!(vis,anim.anim)
        # open(joinpath(graphics_path,prefix,"construction_simulation_rotating.html"),"w") do io
        #     write(io,static_html(vis))
        # end
    end
    return env, vis, anim, STATS
end