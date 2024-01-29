using ConstructionBots

#                                    # parts      # assemblies
#     1 => :colored_8x8                 33               1
#     2 => :quad_nested                 85              21
#     3 => :heavily_nested              1757           508
#     4 => :tractor                     20               8
#     5 => :tie_fighter                 44               4
#     6 => :x_wing_mini                 61              12
#     7 => :imperial_shuttle            84               5
#     8 => :x_wing_tie_mini             105             17
#     9 => :at_te_walker                100             22
#     10 => :x_wing                     309             28
#     11 => :passenger_plane            326             28
#     12 => :imperial_star_destroyer    418             11
#     13 => :kings_castle               761             70
#     14 => :at_at                      1105             2
#     15 => :saturn_v                   1845           306

project_params = get_project_params(4)


open_animation_at_end = true
save_animation_along_the_way = false
save_animation_at_end = false
anim_active_agents = true # green circles around active agents (robots and transport units)
anim_active_areas = true # purple circles around active assembly areas

update_anim_at_every_step = true
save_anim_interval = 100
process_updates_interval = 100
block_save_anim = false

deconflict_strategies = [:RVO, :TangentBugPolicy, :Dispersion]
assignment_mode = :greedy # :milp :milp_w_greedy_warm_start
milp_optimizer = :highs # :gurobi :highs
optimizer_time_limit = 60


env, stats = run_lego_demo(;
    ldraw_file=project_params[:file_name],
    project_name=project_params[:project_name],
    model_scale=project_params[:model_scale],
    num_robots=project_params[:num_robots],
    assignment_mode=assignment_mode,
    milp_optimizer=milp_optimizer,
    optimizer_time_limit=optimizer_time_limit,
    deconflict_strategies=deconflict_strategies,
    open_animation_at_end=open_animation_at_end,
    save_animation=save_animation_at_end,
    save_animation_along_the_way=save_animation_along_the_way,
    anim_active_agents=anim_active_agents,
    anim_active_areas=anim_active_areas,
    update_anim_at_every_step=update_anim_at_every_step,
    save_anim_interval=save_anim_interval,
    process_updates_interval=process_updates_interval,
    block_save_anim=block_save_anim,
    write_results=false,
    overwrite_results=false,
    look_for_previous_milp_solution=false,
    save_milp_solution=false,
    previous_found_optimizer_time=30,
    max_num_iters_no_progress=2500,
    stop_after_task_assignment=false
);
