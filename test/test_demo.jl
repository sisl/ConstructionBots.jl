include("../scripts/full_demo.jl")
include("../scripts/project_params.jl")

using Gurobi

let
    project_params = get_project_params(4)
    open_animation_at_end        = false
    save_animation_along_the_way = false
    save_animation_at_end        = true
    anim_active_agents           = true
    anim_active_areas            = true

    rvo_flag                     = true
    tangent_bug_flag             = true
    dispersion_flag              = true
    assignment_mode              = :greedy # :milp :greedy :milp_w_greedy_warm_start

    write_results                = false
    overwrite_results            = false


    env, stats = run_lego_demo(;
        ldraw_file                   = project_params[:file_name],
        project_name                 = project_params[:project_name],
        model_scale                  = project_params[:model_scale],
        num_robots                   = project_params[:num_robots],

        assignment_mode              = assignment_mode,

        rvo_flag                     = rvo_flag,
        tangent_bug_flag             = tangent_bug_flag,
        dispersion_flag              = dispersion_flag,

        open_animation_at_end        = open_animation_at_end,
        save_animation               = save_animation_at_end,
        save_animation_along_the_way = save_animation_along_the_way,
        anim_active_agents           = anim_active_agents,
        anim_active_areas            = anim_active_areas,

        write_results                = write_results,
        overwrite_results            = overwrite_results,

        stop_after_task_assignment = false
    )

end

let
    project_params = get_project_params(4)
    open_animation_at_end        = false
    save_animation_along_the_way = false
    save_animation_at_end        = false
    anim_active_agents           = false
    anim_active_areas            = false

    rvo_flag                     = false
    tangent_bug_flag             = false
    dispersion_flag              = false
    assignment_mode              = :milp  # :greedy :milp_w_greedy_warm_start

    write_results                = true
    overwrite_results            = true


    env, stats = run_lego_demo(;
        ldraw_file                   = project_params[:file_name],
        project_name                 = project_params[:project_name],
        model_scale                  = project_params[:model_scale],
        num_robots                   = project_params[:num_robots] + 5,

        assignment_mode              = assignment_mode,
        milp_optimizer               = :gurobi, # :gurobi :highs
        optimizer_time_limit         = 30,


        rvo_flag                     = rvo_flag,
        tangent_bug_flag             = tangent_bug_flag,
        dispersion_flag              = dispersion_flag,

        open_animation_at_end        = open_animation_at_end,
        save_animation               = save_animation_at_end,
        save_animation_along_the_way = save_animation_along_the_way,
        anim_active_agents           = anim_active_agents,
        anim_active_areas            = anim_active_areas,

        write_results                = write_results,
        overwrite_results            = overwrite_results,

        look_for_previous_milp_solution = false,
        save_milp_solution              = false,

        stop_after_task_assignment = false
    )

end

let
    project_params = get_project_params(4)
    open_animation_at_end        = false
    save_animation_along_the_way = false
    save_animation_at_end        = false
    anim_active_agents           = false
    anim_active_areas            = false

    rvo_flag                     = false
    tangent_bug_flag             = false
    dispersion_flag              = false
    assignment_mode              = :milp_w_greedy_warm_start

    write_results                = true
    overwrite_results            = true


    env, stats = run_lego_demo(;
        ldraw_file                   = project_params[:file_name],
        project_name                 = project_params[:project_name],
        model_scale                  = project_params[:model_scale],
        num_robots                   = project_params[:num_robots] + 5,

        assignment_mode              = assignment_mode,
        milp_optimizer               = :gurobi, # :gurobi :highs
        optimizer_time_limit         = 30,


        rvo_flag                     = rvo_flag,
        tangent_bug_flag             = tangent_bug_flag,
        dispersion_flag              = dispersion_flag,

        open_animation_at_end        = open_animation_at_end,
        save_animation               = save_animation_at_end,
        save_animation_along_the_way = save_animation_along_the_way,
        anim_active_agents           = anim_active_agents,
        anim_active_areas            = anim_active_areas,

        write_results                = write_results,
        overwrite_results            = overwrite_results,

        look_for_previous_milp_solution = false,
        save_milp_solution              = false,

        stop_after_task_assignment = false
    )

end

let
    include("stage_graph_plots.jl")
end
