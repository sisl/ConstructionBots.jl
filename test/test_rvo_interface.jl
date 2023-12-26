using  ConstructionBots

using Test
using PyCall


@testset "Load RVO Python module success" begin
    ConstructionBots.reset_rvo_python_module!()

    py_mod = ConstructionBots.rvo_python_module()

    @test py_mod !== nothing
    @test typeof(py_mod) == PyCall.PyObject
end

@testset "Create RVO simulation success" begin
    sim = ConstructionBots.rvo_new_sim()

    @test typeof(sim) == PyCall.PyObject
end

@testset "Number of agents initialied to zero" begin
    rvo_map = ConstructionBots.RVOAgentMap()

    # Test initial RVO map has no agents
    @test ConstructionBots.rvo_map_num_agents(rvo_map) == 0
    # TODO: Test adding agents
end

@testset "Add agent success" begin
    scene_tree = ConstructionBots.SceneTree()
    sim = ConstructionBots.rvo_new_sim()

    # Add agents to the simulation
    ConstructionBots.rvo_add_agents!(scene_tree, sim)

    # Test agent propertis have correct types
    for node in ConstructionBots.rvo_active_agents(scene_tree)
        @test typeof(ConstructionBots.rvo_get_agent_position(node)) == Tuple{Float64, Float64}
        @test typeof(ConstructionBots.rvo_get_agent_velocity(node)) == Tuple{Float64, Float64}
        @test typeof(ConstructionBots.rvo_get_agent_pref_velocity(node)) == Tuple{Float64, Float64}
    end
end

@testset "Update agent position success" begin
    scene_tree = ConstructionBots.SceneTree()
    sim = ConstructionBots.rvo_new_sim()
    ConstructionBots.rvo_add_agents!(scene_tree, sim)

    for node in ConstructionBots.rvo_active_agents(scene_tree)
        new_pos = (1.0, 2.0)
        ConstructionBots.rvo_set_agent_position!(node, new_pos)

        # Test agent positions
        @test ConstructionBots.rvo_get_agent_position(node) ≈ new_pos
    end
end

@testset "Update agent velocity success" begin
    scene_tree = ConstructionBots.SceneTree()
    sim = ConstructionBots.rvo_new_sim()
    ConstructionBots.rvo_add_agents!(scene_tree, sim)

    for node in ConstructionBots.rvo_active_agents(scene_tree)
        new_vel = (0.5, 0.5)
        ConstructionBots.rvo_set_agent_pref_velocity!(node, new_vel)

        # Test agent velocities
        @test ConstructionBots.rvo_get_agent_pref_velocity(node) ≈ new_vel
    end
end

# TODO(#33): Add thorough testing before migrating to V1
