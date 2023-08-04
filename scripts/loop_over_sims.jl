
include("full_demo.jl")
include("project_params.jl")
using Gurobi

#                                    # parts      # assemblies
#     1 => :colored_8x8                 33               1                 24
#     2 => :quad_nested                 85              21                 50
#     3 => :heavily_nested              1757           508                 50
#     4 => :tractor                     20               8                 10
#     5 => :tie_fighter                 44               4                 15
#     6 => :x_wing_mini                 61              12                 20
#     7 => :imperial_shuttle            84               5                 20
#     8 => :x_wing_tie_mini             105             17                 20
#     9 => :at_te_walker                100             22                 35
#     10 => :x_wing                     309             28                 50
#     11 => :passenger_plane            326             28                 50
#     12 => :imperial_star_destroyer    418             11                 75
#     13 => :kings_castle               761             70                 125
#     14 => :at_at                      1105             2                 150
#     15 => :saturn_v                   1845           306                 200


# First, run with colored_8x8 for compilation time
project_params = get_project_params(1)
env, stats = run_lego_demo(;
    ldraw_file                   = project_params[:file_name],
    project_name                 = project_params[:project_name],
    model_scale                  = project_params[:model_scale],
    num_robots                   = project_params[:num_robots],
    assignment_mode              = :greedy,
    milp_optimizer               = :gurobi,
    optimizer_time_limit         = 10,
    rvo_flag                     = false,
    dispersion_flag              = false,
    tangent_bug_flag             = false,
    write_results                = false
)


# assignment_modes = [:milp, :milp_w_greedy_warm_start]
# assignment_modes = [:greedy, :milp, :milp_w_greedy_warm_start]
assignment_modes = [:greedy, :milp_w_greedy_warm_start]
# projects_for_sim = [4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15]
projects_for_sim = [13, 15]

rvo_dispersion = [false, true]
tangent_bug = [false, true]
delta_robot = [0]

for proj in projects_for_sim
    for assignment_mode in assignment_modes
        for rvo_disp in rvo_dispersion
            for bug in tangent_bug
                for Δ_robot in delta_robot
                    project_params = get_project_params(proj)
                    try
                        env, stats = run_lego_demo(;
                            ldraw_file                   = project_params[:file_name],
                            project_name                 = project_params[:project_name],
                            model_scale                  = project_params[:model_scale],
                            num_robots                   = project_params[:num_robots] + Δ_robot,

                            assignment_mode              = assignment_mode,
                            milp_optimizer               = :gurobi,
                            optimizer_time_limit         = 3000,

                            rvo_flag                     = rvo_disp,
                            dispersion_flag              = rvo_disp,
                            tangent_bug_flag             = bug,

                            look_for_previous_milp_solution = true,
                            save_milp_solution              = true,
                            previous_found_optimizer_time   = 300
                        )
                    catch err
                        if !isa(err, NoSolutionError)
                            rethrow(err)
                        end
                    end
                end
            end
        end
    end
end
