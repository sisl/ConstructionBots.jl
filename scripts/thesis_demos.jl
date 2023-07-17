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

using ProgressMeter

include("../deps/GraphPlottingBFS.jl")
include("../deps/FactoryRendering.jl")

# Set logging level
log_level = Logging.Info # Logging.Info Logging.Debug Logging.Warn Logging.Error
global_logger(ConsoleLogger(stderr, log_level))

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
ConstructionBots.set_rvo_default_max_speed!(3.0)

## LOAD LDRAW FILE

# project_name                = "colored_8x8.ldr"
# MODEL_SCALE                 = 0.01
# NUM_ROBOTS                  = 25
# OBJECT_VTX_RANGE            = (-10:10,-10:10, 0:1)
# HOME_VTX_RANGE              = (-10:10, -10:10, 0:1)
# MAX_STEPS                   = 3000
# ROBOT_SCALE                 = MODEL_SCALE * 0.9
# visualize_processing        = true
# visualize_animation         = true
# seed                        = 1

# project_name = "ATTEWalker.mpd"
# MODEL_SCALE         = 0.003
# ROBOT_SCALE         = MODEL_SCALE
# NUM_ROBOTS          = 36
# MAX_STEPS           = 8000
# OBJECT_VTX_RANGE    = (-10:10,-10:10, 0:1)
# visualize                   = true
# seed                        = 1

project_name                = "tractor.mpd"
MODEL_SCALE                 = 0.008
ROBOT_SCALE                 = MODEL_SCALE * 0.7
NUM_ROBOTS                  = 12
MAX_STEPS                   = 4000
OBJECT_VTX_RANGE            = (-10:10,-10:10, 0:1)
HOME_VTX_RANGE              = (-10:10, -10:10, 0:1)
STAGING_BUFFER_FACTOR       = 1.5
BUILD_STEP_BUFFER_FACTOR    = 1.5
visualize_processing        = true
visualize_animation         = false
seed                        = 1

# project_name = "X-wingMini.mpd"
# MODEL_SCALE         = 0.007
# ROBOT_SCALE         = MODEL_SCALE * 0.7
# NUM_ROBOTS          = 30
# OBJECT_VTX_RANGE    = (-12:12,-12:12, 0:0)
# HOME_VTX_RANGE      = (-12:12,-12:12, 0:0)
# MAX_STEPS           = 8000
# STAGING_BUFFER_FACTOR = 1.5
# BUILD_STEP_BUFFER_FACTOR = 1.5
# visualize                   = true
# seed                        = 1

# project_name = "quad_nested.mpd"
# MODEL_SCALE               = 0.004
# NUM_ROBOTS                = 50
# ROBOT_SCALE               = MODEL_SCALE
# OBJECT_VTX_RANGE          = (-10:10,-10:10, 0:2)
# MAX_STEPS                 = 3000
# HOME_VTX_RANGE            = (-10:10,-10:10, 0:1)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.55
# visualize                   = true
# seed                        = 1

# project_name = "X-wingFighter.mpd"
# MODEL_SCALE               = 0.0028
# NUM_ROBOTS                = 100
# ROBOT_SCALE               = MODEL_SCALE
# OBJECT_VTX_RANGE          = (-14:0.5:14,-14:0.5:14, 0:0)
# HOME_VTX_RANGE            = (-22:22,-22:22, 0:0)
# MAX_STEPS                 = 8000
# STAGING_BUFFER_FACTOR     = 2.2
# BUILD_STEP_BUFFER_FACTOR  = 0.5
# visualize                   = true
# seed                        = 1

# project_name = "StarDestroyer.mpd"
# MODEL_SCALE               = 0.002
# NUM_ROBOTS                = 100
# # ROBOT_SCALE             = MODEL_SCALE * 0.7
# ROBOT_SCALE               = MODEL_SCALE
# MAX_STEPS                 = 40000
# OBJECT_VTX_RANGE          = (-16:0.5:16,-16:0.5:16, 0:0)
# HOME_VTX_RANGE            = (-24:24,-24:24, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 0.5
# visualize                   = true
# seed                        = 1

# project_name = "Saturn.mpd"
# MODEL_SCALE               = 0.001
# NUM_ROBOTS                = 100
# ROBOT_SCALE               = MODEL_SCALE*4
# MAX_STEPS                 = 100000
# OBJECT_VTX_RANGE          = (-36:36,-36:36,0:8)
# HOME_VTX_RANGE            = (-34:34,-34:34, 0:0)
# STAGING_BUFFER_FACTOR     = 1.5
# BUILD_STEP_BUFFER_FACTOR  = 1.5
# visualize                   = true
# seed                        = 1


# for RVO_FLAG in [true,false]
    # for ASSIGNMENT_MODE in [:OPTIMAL,:GREEDY]
for RVO_FLAG in [true]
    for ASSIGNMENT_MODE in [:OPTIMAL]
        run_lego_demo(;
                project_name            = project_name,
                MODEL_SCALE             = MODEL_SCALE,
                NUM_ROBOTS              = NUM_ROBOTS,
                ROBOT_SCALE             = ROBOT_SCALE,
                OBJECT_VTX_RANGE        = OBJECT_VTX_RANGE,
                HOME_VTX_RANGE          = HOME_VTX_RANGE,
                MAX_STEPS               = MAX_STEPS,
                ASSIGNMENT_MODE         = ASSIGNMENT_MODE,
                RVO_FLAG                = RVO_FLAG,
                visualize_processing    = visualize_processing,
                visualize_animation     = visualize_animation,
                OVERWRITE_RESULTS       = true,
                seed                    = seed
            );
    end
end
