include("full_demo.jl")

## LOAD LDRAW FILE

# project_name = "colored_8x8.ldr" #* 33 x 1, 4 sec
# model_scale  = 0.008
# num_robots   = 24

# project_name = "quad_nested.mpd" #* 85 x 21, 26 sec
# model_scale  = 0.008
# num_robots   = 50

# project_name = "heavily_nested.mpd" #* 1757 x 508, 62 min
# model_scale  = 0.0015
# num_robots   = 50

# project_name = "tractor.mpd" #* 20 x 8, 2 sec
# model_scale  = 0.008
# num_robots   = 10

# project_name = "8028-1 - TIE Fighter - Mini.mpd" #* 44 x 4, 7 sec
# model_scale  = 0.008
# num_robots   = 15

# project_name = "30051-1 - X-wing Fighter - Mini.mpd" #* 61 x 12, 9 sec
# model_scale  = 0.008
# num_robots   = 20

project_name = "4494-1 - Imperial Shuttle - Mini.mpd" #* 84 x 5, 13 sec
model_scale  = 0.008
num_robots   = 15

# project_name = "X-wing--Tie Mini.mpd" #* 105 x 17, 26 sec
# model_scale  = 0.008
# num_robots   = 20

# project_name = "20009-1 - AT-TE Walker - Mini.mpd" #* 100 x 22, 23 sec
# model_scale  = 0.008
# num_robots   = 35

# project_name = "7140-1 - X-wing Fighter.mpd" #* 309 x 28, 3 min
# model_scale  = 0.004
# num_robots   = 50

# project_name = "3181 - Passenger Plane.mpd" #* 326 x 28, 4 min
# model_scale  = 0.004
# num_robots   = 50

# project_name = "8099-1 - Midi-Scale Imperial Star Destroyer.mpd" #* 418 x 11, 5 min
# model_scale  = 0.004
# num_robots   = 75

# project_name = "6080 - Kings Castle.mpd" #* 761 x 70, 21 min
# model_scale  = 0.004
# num_robots   = 125

# project_name = "75054-1 - AT-AT.mpd" #* 1105 x 2
# model_scale  = 0.004
# num_robots   = 100

# TODO again for clean run (with RVO, TangentBug)
# project_name = "21309-1 - NASA Apollo Saturn V.mpd" #* 1845 x 306, 163 min
# model_scale  = 0.0015
# num_robots   = 200

open_animation_at_end        = false
save_animation_along_the_way = false
save_animation_at_end        = false
anim_active_agents           = false
anim_active_areas            = false

rvo_flag                     = false
tangent_bug_flag             = false
dispersion_flag              = false
assignment_mode              = :GreedyWarmStartMILP # :OPTIMAL :GREEDY :GreedyWarmStartMILP

write_results                = true
overwrite_results            = true

max_num_iters_no_progress    = 5000

env, STATS = run_lego_demo(;
    project_name                 = project_name,
    model_scale                  = model_scale,
    num_robots                   = num_robots,
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
    max_num_iters_no_progress    = max_num_iters_no_progress,
    milp_optimizer               = :Gurobi, # :HiGHS
    optimizer_time_limit         = 60
);
