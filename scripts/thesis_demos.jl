include("full_demo.jl")

## LOAD LDRAW FILE

# project_name                = "colored_8x8.ldr"
# model_scale                 = 0.01
# num_robots                  = 24
# object_vtx_range            = (-10:10,-10:10, 0:1)
# home_vtx_range              = (-10:-2, -10:-7, 0:0)
# max_steps                   = 3000
# robot_scale                 = model_scale * 0.9

# project_name              = "quad_nested.mpd"
# model_scale               = 0.004
# num_robots                = 50
# robot_scale               = model_scale
# object_vtx_range          = (-10:10,-10:10, 0:2)
# max_steps                 = 3000
# home_vtx_range            = (-10:10,-10:10, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 0.55

# project_name                = "tractor.mpd"
# model_scale                 = 0.008
# robot_scale                 = model_scale * 0.7
# num_robots                  = 12
# max_steps                   = 3000
# object_vtx_range            = (-10:10,-10:10, 0:1)
# home_vtx_range              = (-10:-4, -10:-8, 0:0)
# staging_buffer_factor       = 1.5
# build_step_buffer_factor    = 1.5

# project_name = "30051-1 - X-wing Fighter - Mini.mpd"
# model_scale               = 0.007
# robot_scale               = model_scale * 0.7
# num_robots                = 30
# max_steps                 = 8000
# object_vtx_range          = (-12:12, -12:12, 0:0)
# home_vtx_range            = (-15:-5, -15:-10, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 1.5

# project_name = "ATTEWalker.mpd"
# model_scale                 = 0.003
# robot_scale                 = model_scale
# num_robots                  = 36
# max_steps                   = 8000
# object_vtx_range            = (-10:10,-10:10, 0:1)
# home_vtx_range              = (-10:-1, -10:-6, 0:0)
# staging_buffer_factor       = 1.5
# build_step_buffer_factor    = 1.5

# project_name = "7140-1 - X-wing Fighter--mod.mpd"
# model_scale               = 0.0028
# robot_scale               = model_scale
# num_robots                = 100 #? Maybe less?
# max_steps                 = 15000
# object_vtx_range          = (-14:0.5:14, -14:0.5:14, 0:0)
# home_vtx_range            = (-25:0, -25:-20, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 0.5

project_name = "6080 - Kings Castle.mpd"
model_scale               = 0.0025
num_robots                = 75
robot_scale               = model_scale
max_steps                 = 40000
object_vtx_range          = (-30:0.5:30, -30:0.5:30, 0:0)
home_vtx_range            = (-35:-20, -35:-20, 0:0)
staging_buffer_factor     = 1.5
build_step_buffer_factor  = 0.5

# project_name = "StarDestroyer.mpd"
# model_scale               = 0.002
# num_robots                = 100
# robot_scale               = model_scale
# max_steps                 = 40000
# object_vtx_range          = (-16:0.5:16, -16:0.5:16, 0:0)
# home_vtx_range            = (-25:0, -25:-20, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 0.5

# project_name = "heavily_nestes.mpd"
# model_scale               = 0.004
# num_robots                = 50
# robot_scale               = model_scale
# object_vtx_range          = (-350:0.5:35,-35:0.5:35, 0:2)
# max_steps                 = 40000
# home_vtx_range            = (-60:0.5:0,-60:0.5:0, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 0.55

# project_name = "Saturn.mpd"
# model_scale               = 0.0015
# num_robots                = 50 #100
# robot_scale               = model_scale
# max_steps                 = 100000
# object_vtx_range          = (-30:0.5:30, -30:0.5:30, 0:1)
# home_vtx_range            = (-50:0.5:0,-50:0, 0:0)
# staging_buffer_factor     = 1.5
# build_step_buffer_factor  = 1.5

visualize_processing         = false
visualize_animation_at_end   = false
save_animation_along_the_way = false
save_animation_at_end        = true
anim_steps                   = true
anim_active_areas            = true
rvo_flag                     = true # false
assignment_mode              = :GREEDY # :OPTIMAL # :GREEDY
write_results                = true
overwrite_results            = true
seed                         = 1

max_num_iters_no_progress    = 1500

using Gurobi

env, STATS = run_lego_demo(;
    project_name                 = project_name,
    model_scale                  = model_scale,
    num_robots                   = num_robots,
    robot_scale                  = robot_scale,
    object_vtx_range             = object_vtx_range,
    home_vtx_range               = home_vtx_range,
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
    seed                         = seed
);
