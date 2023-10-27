
projects = Dict(
    1 => :colored_8x8,                  # 33 x 1, 4 sec
    2 => :quad_nested,                  # 85 x 21, 26 sec
    3 => :heavily_nested,               # 1757 x 508, 62 min
    4 => :tractor,                      # 20 x 8, 2 sec
    5 => :tie_fighter,                  # 44 x 4, 7 sec
    6 => :x_wing_mini,                  # 61 x 12, 9 sec
    7 => :imperial_shuttle,             # 84 x 5, 13 sec
    8 => :x_wing_tie_mini,              # 105 x 17, 26 sec
    9 => :at_te_walker,                 # 100 x 22, 23 sec
    10 => :x_wing,                      # 309 x 28, 3 min
    11 => :passenger_plane,             # 326 x 28, 4 min
    12 => :imperial_star_destroyer,     # 418 x 11, 5 min
    13 => :kings_castle,                # 761 x 70, 21 min
    14 => :at_at,                       # 1105 x 2, ??
    15 => :saturn_v                     # 1845 x 306, 163 min
)

project_parameters = Dict(
    :colored_8x8 => (
        project_name = "colored_8x8",
        file_name = "colored_8x8.ldr",
        model_scale = 0.008,
        num_robots = 24
    ),
    :quad_nested => (
        project_name = "quad_nested",
        file_name = "quad_nested.mpd",
        model_scale = 0.0015,
        num_robots = 50
    ),
    :heavily_nested => (
        project_name = "heavily_nested",
        file_name = "heavily_nested.mpd",
        model_scale = 0.0015,
        num_robots = 50
    ),
    :tractor => (
        project_name = "tractor",
        file_name = "tractor.mpd",
        model_scale = 0.008,
        num_robots = 10
    ),
    :tie_fighter => (
        project_name = "tie_fighter",
        file_name = "8028-1 - TIE Fighter - Mini.mpd",
        model_scale = 0.008,
        num_robots = 15
    ),
    :x_wing_mini => (
        project_name = "x_wing_mini",
        file_name = "30051-1 - X-wing Fighter - Mini.mpd",
        model_scale = 0.008,
        num_robots = 20
    ),
    :imperial_shuttle => (
        project_name = "imperial_shuttle",
        file_name = "4494-1 - Imperial Shuttle - Mini.mpd",
        model_scale = 0.008,
        num_robots = 20
    ),
    :x_wing_tie_mini => (
        project_name = "x_wing_tie_mini",
        file_name = "X-wing--Tie Mini.mpd",
        model_scale = 0.008,
        num_robots = 20
    ),
    :at_te_walker => (
        project_name = "at_te_walker",
        file_name = "20009-1 - AT-TE Walker - Mini.mpd",
        model_scale = 0.008,
        num_robots = 35
    ),
    :x_wing => (
        project_name = "x_wing",
        file_name = "7140-1 - X-wing Fighter.mpd",
        model_scale = 0.004,
        num_robots = 50
    ),
    :passenger_plane => (
        project_name = "passenger_plane",
        file_name = "3181 - Passenger Plane.mpd",
        model_scale = 0.004,
        num_robots = 50
    ),
    :imperial_star_destroyer => (
        project_name = "imperial_star_destroyer",
        file_name = "8099-1 - Midi-Scale Imperial Star Destroyer.mpd",
        model_scale = 0.004,
        num_robots = 75
    ),
    :kings_castle => (
        project_name = "kings_castle",
        file_name = "6080 - Kings Castle.mpd",
        model_scale = 0.004,
        num_robots = 125
    ),
    :at_at => (
        project_name = "at_at",
        file_name = "75054-1 - AT-AT.mpd",
        model_scale = 0.004,
        num_robots = 150
    ),
    :saturn_v => (
        project_name = "saturn_v",
        file_name = "21309-1 - NASA Apollo Saturn V.mpd",
        model_scale = 0.0015,
        num_robots = 200
    )
)

"""
    list_projects()

Prints a list of available projects.
"""
function list_projects()
    println("Available projects:")
    for ii in sort(collect(keys(projects)))
        println("  $ii: $(projects[ii])")
    end
end

"""
    get_project_params(project::Int)
    get_project_params(project::Symbol)

Returns the parameters for the project with the given number or symbol.
Returns a Dict with the following fields:
- `project_name` (String): The name of the project.
- `file_name` (String): The name of the LDraw file.
- `model_scale` (Float64): The default scale of the model.
- `num_robots` (Int): The default number of robots to use.
"""
function get_project_params(project::Int)
    if !(project in keys(projects))
        list_projects()
        error("Project $project not found.")
    end
    return get_project_params(projects[project])
end
function get_project_params(project::Symbol)
    if !(project in keys(project_parameters))
        list_projects()
        error("Project $project not found.")
    end
    return project_parameters[project]
end
