using ConstructionBots

#                                    # parts      # assemblies      # estimated time (rvo+)
#     1 => :colored_8x8                 33               1                   4 sec
#     2 => :quad_nested                 85              21                  26 sec
#     3 => :heavily_nested              1757           508                  62 min
#     4 => :tractor                     20               8                   2 sec
#     5 => :tie_fighter                 44               4                   7 sec
#     6 => :x_wing_mini                 61              12                   9 sec
#     7 => :imperial_shuttle            84               5                  13 sec
#     8 => :x_wing_tie_mini             105             17                  26 sec
#     9 => :at_te_walker                100             22                  23 sec
#     10 => :x_wing                     309             28                   3 min
#     11 => :passenger_plane            326             28                   4 min
#     12 => :imperial_star_destroyer    418             11                   5 min
#     13 => :kings_castle               761             70                  21 min
#     14 => :at_at                      1105             2                  ??
#     15 => :saturn_v                   1845           306                 163 min

project_params = get_project_params(4)


open_animation_at_end        = true
save_animation_along_the_way = false
save_animation_at_end        = false
anim_active_agents           = true
anim_active_areas            = true

tangent_bug_flag             = true
rvo_flag                     = false
dispersion_flag              = false
assignment_mode              = :greedy
# assignment_mode              = :milp
# assignment_mode              = :milp_w_greedy_warm_start

write_results                = false
overwrite_results            = false


env, stats = run_lego_demo(;
    ldraw_file                      = project_params[:file_name],
    project_name                    = project_params[:project_name],
    model_scale                     = project_params[:model_scale],
    num_robots                      = project_params[:num_robots],

    assignment_mode                 = assignment_mode,
    milp_optimizer                  = :gurobi, # :gurobi :highs
    optimizer_time_limit            = 30,

    rvo_flag                        = rvo_flag,
    tangent_bug_flag                = tangent_bug_flag,
    dispersion_flag                 = dispersion_flag,

    open_animation_at_end           = open_animation_at_end,
    save_animation                  = save_animation_at_end,
    save_animation_along_the_way    = save_animation_along_the_way,
    anim_active_agents              = anim_active_agents,
    anim_active_areas               = anim_active_areas,
    update_anim_at_every_step       = false,
    save_anim_interval              = 100,

    write_results                   = write_results,
    overwrite_results               = overwrite_results,

    look_for_previous_milp_solution = false,
    save_milp_solution              = false,
    previous_found_optimizer_time   = 30,

    max_num_iters_no_progress       = 2500,
    stop_after_task_assignment      = false
);
