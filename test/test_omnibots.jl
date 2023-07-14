let
    OmniBotState{2}()
    OmniBotState{3}()
    PlanarRigidBodyState{2}()
    PlanarRigidBodyState{3}()
    PlanarRigidBodyAction{2}()
    PlanarRigidBodyAction{3}()
end
let
    s = OmniBotState{2}(
        pos = [0,0],
        vel = [0.0,0.0],
    )
    a = PlanarRigidBodyAction{2}(
        accel = [1.0,0.0],
        dt = 1.0
    )
    sp = s
    for n in [2,4,8]
        α=π/n
        sp = get_next_state(s,PlanarRigidBodyAction{2}(α=α))
        # @show sp.θ
        for i in 1:(2*n-1)
            sp = get_next_state(sp,PlanarRigidBodyAction{2}())
            # @show sp.θ
        end
        sp = get_next_state(sp,PlanarRigidBodyAction{2}(α=-α))
        # @show sp.θ
        @test isapprox(sp.θ,0.0;rtol=1e-12,atol=1e-12)
    end
end
let
    model = (
        ϵ = 1e-4,
        v_max = 1.0,
        candidate_accels = [one_hot(3,1),one_hot(3,2),-one_hot(3,1),-one_hot(3,2)],
        allow_perpendicular_accel=false,
        allow_translating_rotation=false,
        ω_max = π/2,
        candidate_α_list = [π/2,-π/2],
        dt = 1.0,
    )
    # default_action_space = [OmniBotAction{3}(accel=a,α=α) for a in model.candidate_accels for α in model.candidate_α_list]
    s = OmniBotState{3}()
    for a in get_possible_actions(model,s)
        sp = get_next_state(s,a)
        @show a.α,sp.θ,sp.ω,norm(sp.vel), length(get_possible_actions(model,sp))
    end
end
let
    s = CRCBS.CompositeState(
        (os=OmniBotState{3}(),ps=CRCBS.PlatformState())
    )
    a = CRCBS.CompositeAction(
        (OmniBotAction{3}(),CRCBS.PlatformAction())
    )
    sp = get_next_state(s,a)
end
let
    CRCBS.PlanarRigidBodyAction{3}()
end
