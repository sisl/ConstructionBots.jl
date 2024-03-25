using Test
using PyCall
using Parameters
using Graphs, MetaGraphs
using ConstructionBots

let
    @test ConstructionBots.ReciprocalVelocityObstacle() isa ConstructionBots.DeconflictStrategy
    @test ConstructionBots.TangentBugPolicy() isa ConstructionBots.DeconflictStrategy
    @test ConstructionBots.PotentialFields() isa ConstructionBots.DeconflictStrategy
    @test ConstructionBots.CombinedRVOPolicy() isa ConstructionBots.DeconflictStrategy
    @test ConstructionBots.NoDeconfliction() isa ConstructionBots.DeconflictStrategy 
end

let
    strategies = [
        ConstructionBots.ReciprocalVelocityObstacle(),
        ConstructionBots.TangentBugPolicy(),
        ConstructionBots.PotentialFields(),
        ConstructionBots.CombinedRVOPolicy(),
        ConstructionBots.NoDeconfliction()
        ]
end

# Mock definitions

mutable struct RobotNode
    id::Int
    desired_twist::Tuple{Float64, Float64}
    alpha::Float64
end

function get_agent_max_speed(agent::RobotNode)
    return DEFAULT_MAX_SPEED
end

function entity(node)
    return RobotNode(1, (0.0, 0.0), 0.5)
end

function global_transform(node)
    return (x=0.0, y=0.0, z=0.0)
end

function goal_config(node)
    return (x=1.0, y=1.0, z=0.0)
end

function compute_twist_from_goal(env, agent, goal, dt)
    return (1.0, 1.0) # Linear and angular velocity
end

let
    rvo = ConstructionBots.ReciprocalVelocityObstacle()
    @test rvo.name == "ReciprocalVelocityObstacle"
    @test rvo.dt == 1 / 40.0
    @test rvo.max_speed == 4.0
    @test rvo.max_speed_volume_factor == 0.01
    @test rvo.min_max_speed == 1.0
    @test rvo.default_time_step == 1 / 40.0
    @test rvo.neighbor_distance == 2.0
    @test rvo.min_neighbor_distance == 1.0
    @test rvo.neighborhood_velocity_scale_factor == 1.0
end

let
    rvo = ConstructionBots.ReciprocalVelocityObstacle()
    env = PlannerEnv(dt=1/40.0, max_robot_go_id=50, max_cargo_id=50)  # Mock environment
    # node = RobotGo(RobotNode(1, GeomNode(nothing)))  # Mock node
    # twist = ConstructionBots.perform_twist_deconfliction(rvo, env, node)
    # goal = global_transform(goal_config(node))
    # dt = 1/40.0
    # @test twist == compute_twist_from_goal(env, node, goal, dt)
end
