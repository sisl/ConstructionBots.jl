using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using TaskGraphs
using JuMP
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)

using LightGraphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations
using StaticArrays
using LinearAlgebra

using PyCall
rvo = pyimport("rvo2")
ConstructionBots.set_rvo_python_module!(rvo)
ConstructionBots.set_rvo_default_max_speed!(3.0)

using MeshCat
const MESHCAT_GRID_DIMS = ((-10.0,10.0),(-10.0,10.0))
using Plots
using Random

using TOML
using Logging
global_logger(ConsoleLogger(stderr, Logging.Info))
# global_logger(ConsoleLogger(stderr, Logging.Debug))

Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))
Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers","render_tools.jl"))
Revise.includet(joinpath(pathof(ConstructionBots),"../..","scripts","full_demo.jl"))

TaskGraphs.set_default_optimizer_attributes!(
    "TimeLimit"=>100,
    MOI.Silent()=>false
    )

vis = MeshCat.Visualizer()
MeshCat.render(vis)

# Start MeshCat viewer
reset_all_id_counters!()
reset_all_invalid_id_counters!()
Random.seed!(0);

## LOAD LDRAW FILE
# project_name = "simple_quad_stack.mpd"
# project_name = "DemoStack.mpd"
# project_name = "ATTEWalker.mpd"
# project_name = "stack1.ldr"
# project_name = "big_stack.ldr"
# project_name = "triple_stack.mpd"

# project_name = "ATTEWalker.mpd"
# MODEL_SCALE         = 0.003
# NUM_ROBOTS          = 36
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)

# project_name = "tractor.mpd"
# MODEL_SCALE         = 0.01
# ROBOT_SCALE         = MODEL_SCALE * 0.7
# NUM_ROBOTS          = 12
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE      = (-10:10,-10:10, 0:1)
# MAX_STEPS           = 3000

project_name = "X-wingMini.mpd"
MODEL_SCALE         = 0.01
ROBOT_SCALE         = MODEL_SCALE * 0.7
NUM_ROBOTS          = 30
OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)
HOME_VTX_RANGE      = (-10:10,-10:10, 0:1)
MAX_STEPS           = 5000
STAGING_BUFFER_FACTOR = 0.5
BUILD_STEP_BUFFER_FACTOR = 0.25

# project_name = "quad_nested.mpd"
# MODEL_SCALE         = 0.004
# NUM_ROBOTS          = 50
# ROBOT_SCALE         = MODEL_SCALE
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:2)
# MAX_STEPS           = 3000
# HOME_VTX_RANGE      = (-10:10,-10:10, 0:1)
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 0.55

project_name = "X-wingFighter.mpd"
MODEL_SCALE         = 0.004
NUM_ROBOTS          = 100
ROBOT_SCALE         = MODEL_SCALE
OBJECT_VTX_RANGE    = (-26:26,-26:26, 0:10)
HOME_VTX_RANGE    = (-26:26,-26:26, 0:0)
MAX_STEPS           = 20000
STAGING_BUFFER_FACTOR = 1.5
BUILD_STEP_BUFFER_FACTOR = 0.5

# project_name = "StarDestroyer.mpd"
# MODEL_SCALE         = 0.004
# NUM_ROBOTS          = 100
# ROBOT_SCALE         = MODEL_SCALE * 0.7
# MAX_STEPS           = 20000
# OBJECT_VTX_RANGE    = (-20:20,-20:20, 0:5)
# HOME_VTX_RANGE    = (-26:26,-26:26, 0:0)
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 0.5

project_name = "Saturn.mpd"
MODEL_SCALE         = 0.001
NUM_ROBOTS          = 100
ROBOT_SCALE         = MODEL_SCALE*4
MAX_STEPS           = 20000
OBJECT_VTX_RANGE =(-36:36,-36:36,0:8)
HOME_VTX_RANGE    = (-34:34,-34:34, 0:0)
STAGING_BUFFER_FACTOR = 1.5
BUILD_STEP_BUFFER_FACTOR = 0.5

# project_name = "colored_8x8.ldr"
# MODEL_SCALE         = 0.01
# ROBOT_SCALE         = MODEL_SCALE * 0.9
# NUM_ROBOTS          = 25
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)

for RVO_FLAG in [true,false]
    # for ASSIGNMENT_MODE in [:OPTIMAL,:GREEDY]
    for ASSIGNMENT_MODE in [:GREEDY]
        run_lego_demo(;
                project_name          = project_name,
                MODEL_SCALE         = MODEL_SCALE,
                NUM_ROBOTS          = NUM_ROBOTS,
                ROBOT_SCALE         = ROBOT_SCALE,
                OBJECT_VTX_RANGE    = OBJECT_VTX_RANGE,
                HOME_VTX_RANGE      = HOME_VTX_RANGE,
                MAX_STEPS           = MAX_STEPS,
                ASSIGNMENT_MODE     = ASSIGNMENT_MODE,
                RVO_FLAG            = RVO_FLAG,
                VISUALIZER          = false,
                vis                 = nothing,
                # vis                 = vis,
            );
    end
end


run_lego_demo(;
        project_name        = project_name,
        MODEL_SCALE         = MODEL_SCALE,
        NUM_ROBOTS          = NUM_ROBOTS,
        ROBOT_SCALE         = ROBOT_SCALE,
        OBJECT_VTX_RANGE    = OBJECT_VTX_RANGE,
        HOME_VTX_RANGE      = HOME_VTX_RANGE,
        MAX_STEPS           = MAX_STEPS,
        ASSIGNMENT_MODE     = :GREEDY,
        RVO_FLAG            = false,
        VISUALIZER          = false,
        vis                 = nothing,
        WRITE_RESULTS       = false,
    );