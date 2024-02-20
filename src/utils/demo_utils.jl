
struct SimParameters
    sim_batch_size::Int
    max_time_steps::Int
    process_animation_tasks::Bool
    save_anim_interval::Int
    process_updates_interval::Int
    block_save_anim::Bool
    update_anim_at_every_step::Bool
    anim_active_agents::Bool
    anim_active_areas::Bool
    save_anim_prog_path::String
    save_animation_along_the_way::Bool
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

struct NoSolutionError <: Exception end
Base.showerror(io::IO, e::NoSolutionError) = print(io, "No solution found!")

"""
    run_simulation!(env, factory_vis, anim, sim_params)

Run a simulation of the environment. Currently, this wraps another simulation function to
help with memory issues. The goal is to refactor the code to not require this "hack".
"""
function run_simulation!(
    env::PlannerEnv,
    factory_vis::ConstructionBots.FactoryVisualizer,
    anim::Union{ConstructionBots.AnimationWrapper,Nothing},
    sim_params::SimParameters,
)
    it = 1
    ConstructionBots.step_environment!(env)
    ConstructionBots.update_planning_cache!(env, 0.0)

    step_1_closed = length(env.cache.closed_set)
    total_nodes = nv(env.sched)
    generate_showvalues(it, nc) =
        () -> [(:step_num, it), (:n_closed, nc), (:n_total, total_nodes)]
    prog = ProgressMeter.Progress(
        nv(env.sched) - step_1_closed;
        desc = "Simulating...",
        barlen = 50,
        showspeed = true,
        dt = 1.0,
    )
    starting_frame = 0
    if !isnothing(anim)
        starting_frame = ConstructionBots.current_frame(anim)
    end
    sim_process_data = SimProcessingData(
        false,
        1,
        starting_frame,
        prog,
        step_1_closed,
        step_1_closed,
        0,
        0,
        generate_showvalues,
    )
    up_steps = []
    while !sim_process_data.stop_simulating &&
        sim_process_data.iter < sim_params.max_time_steps
        up_steps = simulate!(env, factory_vis, anim, sim_params, sim_process_data, up_steps)
    end

    return ConstructionBots.project_complete(env), sim_process_data.iter
end

function simulate!(
    env::PlannerEnv,
    factory_vis::ConstructionBots.FactoryVisualizer,
    anim::Union{ConstructionBots.AnimationWrapper,Nothing},
    sim_params::SimParameters,
    sim_process_data::SimProcessingData,
    update_steps::Vector,
)
    @unpack sched, cache = env
    @unpack sim_batch_size, max_time_steps = sim_params
    @unpack process_animation_tasks, save_anim_interval = sim_params
    @unpack process_updates_interval, block_save_anim, anim_active_agents = sim_params
    @unpack anim_active_areas, max_num_iters_no_progress = sim_params
    @unpack save_animation_along_the_way, save_anim_prog_path = sim_params

    for _ = 1:sim_batch_size
        sim_process_data.iter += 1
        ConstructionBots.step_environment!(env)
        newly_updated = ConstructionBots.update_planning_cache!(env, 0.0)

        if !isnothing(factory_vis.vis)
            if !isempty(newly_updated) || sim_params.update_anim_at_every_step
                scene_nodes,
                closed_steps_nodes,
                active_build_nodes,
                fac_active_flags_nodes = ConstructionBots.visualizer_update_function!(
                    factory_vis,
                    env,
                    newly_updated,
                )
                sim_process_data.num_iters_since_anim_save += 1
                if process_animation_tasks
                    update_tuple = (
                        sim_process_data.iter,
                        deepcopy(factory_vis.vis_nodes),
                        deepcopy(scene_nodes),
                        closed_steps_nodes,
                        active_build_nodes,
                        fac_active_flags_nodes,
                    )
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
            if block_save_anim
                anim = ConstructionBots.AnimationWrapper(0)
                update_visualizer!(factory_vis)
                settransform!(
                    factory_vis.vis["/Cameras/default"],
                    CoordinateTransformations.Translation(0, 0, 0),
                )
                setprop!(
                    factory_vis.vis["/Cameras/default/rotated/<object>"],
                    "position",
                    CoordinateTransformations.Translation(0, 10, 0).translation,
                )
            end
            k_i = 0
            for step_k in update_steps
                k_k = step_k[1]
                vis_nodes_k = step_k[2]
                scene_nodes_k = step_k[3]
                closed_steps_nodes_k = step_k[4]
                active_build_nodes_k = step_k[5]
                fac_active_flags_nodes_k = step_k[6]

                current_frame = k_k + sim_process_data.starting_frame
                if block_save_anim
                    k_i += 1
                    current_frame = k_i
                end
                ConstructionBots.set_current_frame!(anim, current_frame)
                atframe(anim, ConstructionBots.current_frame(anim)) do
                    if anim_active_areas
                        for node_i in closed_steps_nodes_k
                            setvisible!(node_i, false)
                        end
                        for node_i in active_build_nodes_k
                            setvisible!(node_i, true)
                        end
                    end
                    if anim_active_agents
                        setvisible!(factory_vis.active_flags, false)
                        for node_key in fac_active_flags_nodes_k
                            setvisible!(factory_vis.active_flags[node_key], true)
                        end
                    end
                    ConstructionBots.update_visualizer!(vis_nodes_k, scene_nodes_k)
                end
            end
            setanimation!(factory_vis.vis, anim.anim, play = false)
            update_steps = []
            if (
                save_animation_along_the_way &&
                !isnothing(save_anim_prog_path) &&
                !project_stop_bool &&
                sim_process_data.num_iters_since_anim_save >= save_anim_interval
            )
                save_animation!(
                    factory_vis.vis,
                    "$(save_anim_prog_path)$(sim_process_data.iter).html",
                )
                sim_process_data.num_iters_since_anim_save = 0
            end
        end

        nc = length(cache.closed_set)
        ProgressMeter.update!(
            sim_process_data.prog,
            nc - sim_process_data.num_closed_step_1;
            showvalues = sim_process_data.progress_update_fcn(sim_process_data.iter, nc),
        )
        if project_stop_bool
            ProgressMeter.finish!(sim_process_data.prog)
            if ConstructionBots.project_complete(env)
                println("PROJECT COMPLETE!")
            else
                println("PROJECT INCOMPLETE!")
                var_dump_path =
                    joinpath(dirname(pathof(ConstructionBots)), "..", "variable_dump")
                mkpath(var_dump_path)
                var_dump_path =
                    joinpath(var_dump_path, "var_dump_$(sim_process_data.iter).jld2")
                println("Dumping variables to $(var_dump_path)")
                var_dict = Dict(
                    "env" => env,
                    "factory_vis" => factory_vis,
                    "anim" => anim,
                    "sim_params" => sim_params,
                    "sim_process_data" => sim_process_data,
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
    circles = Dict{AbstractID,LazySets.Ball2}()
    for n in node_iterator(sched, topological_sort_by_dfs(sched))
        if matches_template(OpenBuildStep, n)
            id = node_id(n)
            sphere = get_cached_geom(node_val(n).staging_circle)
            ctr = project_to_2d(sphere.center)
            circles[id] = LazySets.Ball2(ctr, sphere.radius)
        end
    end
    return circles
end

function shift_staging_circles(
    staging_circles::Dict{AbstractID,LazySets.Ball2},
    sched,
    scene_tree,
)
    shifted_circles = Dict{AbstractID,LazySets.Ball2}()
    for (id, geom) in staging_circles
        node = get_node(scene_tree, id)
        cargo = get_node(scene_tree, id)
        if isa(cargo, AssemblyNode)
            cargo_node = get_node(sched, AssemblyComplete(cargo))
        else
            cargo_node = get_node!(sched, ObjectStart(cargo, TransformNode()))
        end
        translate = project_to_2d(global_transform(start_config(cargo_node)).translation)
        tform = CoordinateTransformations.Translation(translate)
        new_center = tform(geom.center)
        shifted_circles[id] = LazySets.Ball2(new_center, geom.radius)
    end
    return shifted_circles
end

function remove_redundant(balls::Vector{LazySets.Ball2}; ϵ = 1e-4)
    redundant = falses(length(balls))
    for ii = 1:length(balls)
        for jj = 1:length(balls)
            if ii != jj && !redundant[jj] && contained_in(balls[ii], balls[jj]; ϵ = ϵ)
                redundant[ii] = true
            end
        end
    end
    return balls[.!redundant]
end

function contained_in(test_ball::LazySets.Ball2, outer_ball::LazySets.Ball2; ϵ = 1e-4)
    return norm(test_ball.center - outer_ball.center) + test_ball.radius <=
           outer_ball.radius + ϵ
end

function get_object_vtx(scene_tree, obstacles, max_cargo_height, num_layers, robot_d)
    # Use object height to stack objects on top of each other as needed
    objects_hyperrects = map(
        n -> get_base_geom(n, HyperrectangleKey()),
        filter(n -> matches_template(ObjectNode, n), get_nodes(scene_tree)),
    )
    max_object_x = maximum([ob.radius[1] * 2 for ob in objects_hyperrects])
    max_object_y = maximum([ob.radius[2] * 2 for ob in objects_hyperrects])
    max_object_h = maximum([ob.radius[3] * 2 for ob in objects_hyperrects])
    mean_object_x =
        sum([ob.radius[1] * 2 for ob in objects_hyperrects]) / length(objects_hyperrects)
    mean_object_y =
        sum([ob.radius[2] * 2 for ob in objects_hyperrects]) / length(objects_hyperrects)

    std_object_x = std([ob.radius[1] * 2 for ob in objects_hyperrects])
    std_object_y = std([ob.radius[2] * 2 for ob in objects_hyperrects])
    num_objects =
        length(filter(n -> matches_template(ObjectNode, n), get_nodes(scene_tree)))
    object_x_delta = min(max_object_x, mean_object_x + 3 * std_object_x) + robot_d * 2
    object_y_delta = min(max_object_y, mean_object_y + 3 * std_object_y) + robot_d * 2

    inflate_delta = max(max_object_x, max_object_y)
    inflated_obstacles = []
    for ob in obstacles
        push!(inflated_obstacles, LazySets.Ball2(ob.center, ob.radius + inflate_delta))
    end

    num_objs_per_layer = ceil(num_objects / num_layers)
    init_width = sqrt(num_objs_per_layer * object_x_delta * object_y_delta) / 2
    x_range = -init_width:object_x_delta:init_width
    y_range = -init_width:object_y_delta:init_width
    sweep_ranges = (x_range, y_range, 0:0)
    found_size = false
    cnt = 0
    while !found_size
        pts = ConstructionBots.construct_vtx_array(;
            origin = SVector{3,Float64}(0.0, 0.0, 0.0),
            obstacles = inflated_obstacles,
            ranges = sweep_ranges,
        )
        if length(pts) * num_layers >= num_objects
            found_size = true
        else
            cnt += 1
            x_range =
                (-init_width-cnt*object_x_delta):object_x_delta:(init_width+cnt*object_x_delta)
            y_range =
                (-init_width-cnt*object_y_delta):object_y_delta:(init_width+cnt*object_y_delta)
            sweep_ranges = (x_range, y_range, 0:0)
        end
    end
    object_vtx_range = (x_range, y_range, 0:num_layers-1)
    vtxs = ConstructionBots.construct_vtx_array(;
        origin = SVector{3,Float64}(0.0, 0.0, max_cargo_height),
        obstacles = inflated_obstacles,
        ranges = object_vtx_range,
        spacing = (1.0, 1.0, max_object_h),
    )

    return vtxs
end
