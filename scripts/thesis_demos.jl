using ConstructionBots
using LDrawParser
using HierarchicalGeometry
using LazySets

using TaskGraphs
using JuMP

using Graphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations
using StaticArrays
using LinearAlgebra

using PyCall

using MeshCat
using Plots
using Random

using TOML
using Logging

using Colors
using Printf
using Parameters

using PGFPlots
using PGFPlotsX
using Measures
import Cairo
using Compose

using JLD2

using ProgressMeter

# Set logging level
log_level = Logging.LogLevel(2) # Logging.LogLvel(-1) Logging.Info Logging.Debug Logging.Warn Logging.Error
global_logger(ConsoleLogger(stderr, log_level))

include("../deps/GraphPlottingBFS.jl")
include("../deps/FactoryRendering.jl")

include("../src/render_tools.jl")
include("../src/tg_render_tools.jl")
include("full_demo.jl")


using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)

TaskGraphs.set_default_optimizer_attributes!(
    "TimeLimit"=>100,
    MOI.Silent()=>false
    )

rvo = pyimport("rvo2")
ConstructionBots.set_rvo_python_module!(rvo)

## LOAD LDRAW FILE

# project_name                = "colored_8x8.ldr"
# MODEL_SCALE                 = 0.01
# NUM_ROBOTS                  = 24
# OBJECT_VTX_RANGE            = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE              = (-10:-2, -10:-7, 0:0)
# MAX_STEPS                   = 3000
# ROBOT_SCALE                 = MODEL_SCALE * 0.9

# project_name              = "quad_nested.mpd"
# MODEL_SCALE               = 0.004
# NUM_ROBOTS                = 50
# ROBOT_SCALE               = MODEL_SCALE
# OBJECT_VTX_RANGE          = (-10:10,-10:10, 0:2)
# MAX_STEPS                 = 3000
# HOME_VTX_RANGE            = (-10:10,-10:10, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.55

# project_name              = "quad_nested.mpd"
# MODEL_SCALE               = 0.004
# NUM_ROBOTS                = 50
# ROBOT_SCALE               = MODEL_SCALE
# OBJECT_VTX_RANGE          = (-70:0.5:20,-70:0.5:20, 0:2)
# MAX_STEPS                 = 3000
# HOME_VTX_RANGE            = (-70:0,-70:0, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.55

# project_name                = "tractor.mpd"
# MODEL_SCALE                 = 0.008
# ROBOT_SCALE                 = MODEL_SCALE * 0.7
# NUM_ROBOTS                  = 12
# MAX_STEPS                   = 3000
# OBJECT_VTX_RANGE            = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE              = (-10:-4, -10:-8, 0:0)
# STAGING_BUFFER_FACTOR       = 1.5
# BUILD_STEP_BUFFER_FACTOR    = 1.5

# project_name = "X-wingMini.mpd"
# MODEL_SCALE               = 0.007
# ROBOT_SCALE               = MODEL_SCALE * 0.7
# NUM_ROBOTS                = 30
# MAX_STEPS                 = 8000
# OBJECT_VTX_RANGE          = (-12:12, -12:12, 0:0)
# HOME_VTX_RANGE            = (-15:-5, -15:-10, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 1.5

# project_name = "ATTEWalker.mpd"
# MODEL_SCALE                 = 0.003
# ROBOT_SCALE                 = MODEL_SCALE
# NUM_ROBOTS                  = 36
# MAX_STEPS                   = 8000
# OBJECT_VTX_RANGE            = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE              = (-10:-1, -10:-6, 0:0)
# STAGING_BUFFER_FACTOR       = 1.5
# BUILD_STEP_BUFFER_FACTOR    = 1.5


# project_name = "X-wingFighter.mpd"
# MODEL_SCALE               = 0.0028
# ROBOT_SCALE               = MODEL_SCALE
# NUM_ROBOTS                = 100
# MAX_STEPS                 = 8000
# OBJECT_VTX_RANGE          = (-14:0.5:14, -14:0.5:14, 0:0)
# HOME_VTX_RANGE            = (-25:0, -25:-20, 0:0)
# STAGING_BUFFER_FACTOR     = 2.2
# BUILD_STEP_BUFFER_FACTOR  = 0.5

# project_name = "StarDestroyer.mpd"
# MODEL_SCALE               = 0.002
# NUM_ROBOTS                = 100
# ROBOT_SCALE               = MODEL_SCALE
# MAX_STEPS                 = 40000
# OBJECT_VTX_RANGE          = (-16:0.5:16, -16:0.5:16, 0:0)
# HOME_VTX_RANGE            = (-25:0, -25:-20, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.5

# project_name                = "heavily_nested.mpd"
# MODEL_SCALE               = 0.004
# NUM_ROBOTS                = 50
# ROBOT_SCALE               = MODEL_SCALE
# OBJECT_VTX_RANGE          = (-350:0.5:35,-35:0.5:35, 0:2)
# MAX_STEPS                 = 40000
# HOME_VTX_RANGE            = (-60:0.5:0,-60:0.5:0, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.55

project_name = "Saturn.mpd"
MODEL_SCALE               = 0.0015
NUM_ROBOTS                = 100
ROBOT_SCALE               = MODEL_SCALE
MAX_STEPS                 = 100000
OBJECT_VTX_RANGE          = (-30:0.5:30, -30:0.5:30, 0:1)
HOME_VTX_RANGE            = (-50:0.5:0,-50:0, 0:0)
STAGING_BUFFER_FACTOR     = 1.5
BUILD_STEP_BUFFER_FACTOR  = 1.5



visualize_processing         = false
visualize_animation_at_end   = true
save_animation_along_the_way = true
save_animation_at_end        = false
anim_steps                   = true
anim_active_areas            = true
RVO_FLAG                     = true # false
ASSIGNMENT_MODE              = :GREEDY # :OPTIMAL # :GREEDY
OVERWRITE_RESULTS            = true
seed                         = 1

max_num_iters_no_progress    = 2500


env, STATS = run_lego_demo(;
    project_name                 = project_name,
    MODEL_SCALE                  = MODEL_SCALE,
    NUM_ROBOTS                   = NUM_ROBOTS,
    ROBOT_SCALE                  = ROBOT_SCALE,
    OBJECT_VTX_RANGE             = OBJECT_VTX_RANGE,
    HOME_VTX_RANGE               = HOME_VTX_RANGE,
    MAX_STEPS                    = MAX_STEPS,
    ASSIGNMENT_MODE              = ASSIGNMENT_MODE,
    RVO_FLAG                     = RVO_FLAG,
    visualize_processing         = visualize_processing,
    visualize_animation_at_end   = visualize_animation_at_end,
    save_animation               = save_animation_at_end,
    save_animation_along_the_way = save_animation_along_the_way,
    anim_steps                   = anim_steps,
    anim_active_areas            = anim_active_areas,
    OVERWRITE_RESULTS            = OVERWRITE_RESULTS,
    max_num_iters_no_progress    = max_num_iters_no_progress,
    seed                         = seed
);
