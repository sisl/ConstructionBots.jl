include("full_demo.jl")
include("project_params.jl")

#! These need to be included based on the optimizer choice
using Gurobi
# using GLPK
# using HiGHS

# projects = Dict(
#     1 => :colored_8x8,                  # 33 x 1, 4 sec
#     2 => :quad_nested,                  # 85 x 21, 26 sec
#     3 => :heavily_nested,               # 1757 x 508, 62 min
#     4 => :tractor,                      # 20 x 8, 2 sec
#     5 => :tie_fighter,                  # 44 x 4, 7 sec
#     6 => :x_wing_mini,                  # 61 x 12, 9 sec
#     7 => :imperial_shuttle,             # 84 x 5, 13 sec
#     8 => :x_wing_tie_mini,              # 105 x 17, 26 sec
#     9 => :at_te_walker,                 # 100 x 22, 23 sec
#     10 => :x_wing,                      # 309 x 28, 3 min
#     11 => :passenger_plane,             # 326 x 28, 4 min
#     12 => :imperial_star_destroyer,     # 418 x 11, 5 min
#     13 => :kings_castle,                # 761 x 70, 21 min
#     14 => :at_at,                       # 1105 x 2, ??
#     15 => :saturn_v                     # 1845 x 306, 163 min
# )

project_params = get_project_params(4)


open_animation_at_end        = false
save_animation_along_the_way = false
save_animation_at_end        = false
anim_active_agents           = false
anim_active_areas            = false

rvo_flag                     = false
tangent_bug_flag             = false
dispersion_flag              = false
assignment_mode              = :greedy # :milp :greedy :milp_w_greedy_warm_start

write_results                = true
overwrite_results            = true


env, STATS = run_lego_demo(;
    project_name                 = project_params[:file_name],
    model_scale                  = project_params[:model_scale],
    num_robots                   = project_params[:num_robots],

    assignment_mode              = assignment_mode,
    milp_optimizer               = :Gurobi, # :HiGHS
    optimizer_time_limit         = 60,

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

);