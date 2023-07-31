include("full_demo.jl")

## LOAD LDRAW FILE

# project_name = "colored_8x8.ldr" #* 33 x 1
# model_scale                 = 0.008
# robot_scale                 = model_scale * 0.7
# num_robots                  = 24
# max_steps                   = 3000
# staging_buffer_factor       = 1.2
# build_step_buffer_factor    = 0.5

# project_name = "quad_nested.mpd" #* 85 x 21
# model_scale               = 0.008
# robot_scale               = model_scale * 0.7
# num_robots                = 50
# max_steps                 = 3000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "heavily_nested.mpd" #* 1757 x 508
# model_scale               = 0.008
# num_robots                = 50
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "tractor.mpd" #* 20 x 8
# model_scale                 = 0.008
# robot_scale                 = model_scale * 0.7
# num_robots                  = 10
# max_steps                   = 5000
# staging_buffer_factor       = 1.2
# build_step_buffer_factor    = 0.5

# project_name = "8028-1 - TIE Fighter - Mini.mpd" #* 44 x 4
# model_scale               = 0.008
# num_robots                = 15
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "4494-1 - Imperial Shuttle - Mini.mpd" #* 84 x 5
# model_scale               = 0.008 # 0.0028
# robot_scale               = model_scale * 0.7
# num_robots                = 15
# max_steps                 = 15000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "30051-1 - X-wing Fighter - Mini.mpd" #* 61 x 12
# model_scale               = 0.008
# robot_scale               = model_scale * 0.7
# num_robots                = 20
# max_steps                 = 8000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "X-wing--Tie Mini.mpd" #* 105 x 17
# model_scale               = 0.008
# num_robots                = 20
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "20009-1 - AT-TE Walker - Mini.mpd" #* 100 x 22
# model_scale                 = 0.008
# robot_scale                 = model_scale * 0.7
# num_robots                  = 35
# max_steps                   = 8000
# staging_buffer_factor       = 1.2
# build_step_buffer_factor    = 0.5

# project_name = "7140-1 - X-wing Fighter.mpd" #* 309 x 28
# model_scale               = 0.008 # 0.0028
# robot_scale               = model_scale * 0.7
# num_robots                = 50
# max_steps                 = 15000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "3181 - Passenger Plane.mpd" #* 326 x 28
# model_scale               = 0.004
# num_robots                = 50
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "8099-1 - Midi-Scale Imperial Star Destroyer.mpd" #* 418 x 11
# model_scale               = 0.004
# num_robots                = 75
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "6080 - Kings Castle.mpd" #* 761 x 70
# model_scale               = 0.004
# num_robots                = 125
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "75054-1 - AT-AT.mpd" #* 1105 x 2
# model_scale               = 0.004
# num_robots                = 100
# robot_scale               = model_scale * 0.7
# max_steps                 = 40000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

# project_name = "21309-1 - NASA Apollo Saturn V.mpd" #* 1845 x 306
# model_scale               = 0.0015
# num_robots                = 200
# robot_scale               = model_scale * 0.7
# max_steps                 = 100000
# staging_buffer_factor     = 1.2
# build_step_buffer_factor  = 0.5

visualize_processing         = false
visualize_animation_at_end   = true
save_animation_along_the_way = true
save_animation_at_end        = true
anim_steps                   = true
anim_active_areas            = true
rvo_flag                     = true # false
assignment_mode              = :GREEDY # :OPTIMAL # :GREEDY
write_results                = true
overwrite_results            = true
seed                         = 1

num_object_layers            = 1
max_num_iters_no_progress    = 10000

sim_batch_size               = 50
process_updates_interval     = 25
save_anim_interval           = 100

using Gurobi

env, STATS = run_lego_demo(;
    project_name                 = project_name,
    model_scale                  = model_scale,
    num_robots                   = num_robots,
    robot_scale                  = robot_scale,
    num_object_layers            = num_object_layers,
    max_steps                    = max_steps,
    assignment_mode              = assignment_mode,
    rvo_flag                     = rvo_flag,
    visualize_processing         = visualize_processing,
    visualize_animation_at_end   = visualize_animation_at_end,
    save_animation               = save_animation_at_end,
    save_animation_along_the_way = save_animation_along_the_way,
    anim_steps                   = anim_steps,
    anim_active_areas            = anim_active_areas,
    write_results                = write_results,
    overwrite_results            = overwrite_results,
    max_num_iters_no_progress    = max_num_iters_no_progress,
    default_milp_optimizer       = Gurobi.Optimizer,
    sim_batch_size               = sim_batch_size,
    process_updates_interval     = process_updates_interval,
    save_anim_interval           = save_anim_interval,
    seed                         = seed
);
