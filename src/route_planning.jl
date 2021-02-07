export 
    PlannerEnv,
    step_environment!,
    get_cmd,
    apply_cmd!

"""
    PlannerEnv

Contains the Environment state and definition.
"""
@with_kw struct PlannerEnv
    sched::OperatingSchedule    = OperatingSchedule()
    scene_tree::SceneTree       = SceneTree()
    cache::PlanningCache        = initialize_planning_cache(sched)
    dt::Float64                 = rvo_default_time_step()
end

""" 
    LOADING_SPEED

The max velocity with which a part may be loaded (e.g., by LiftIntoPlace,
FormTransportUnit,DepositCargo)
"""
global LOADING_SPEED = 0.1
default_loading_speed() = LOADING_SPEED
function set_default_loading_speed!(val)
    global LOADING_SPEED = val
end

""" 
    ROTATIONAL_LOADING_SPEED 

The max rotational velocity with which a part may be loaded (e.g., by 
LiftIntoPlace,FormTransportUnit,DepositCargo)
"""
global ROTATIONAL_LOADING_SPEED = 0.1
default_rotational_loading_speed() = ROTATIONAL_LOADING_SPEED
function set_default_rotational_loading_speed!(val)
    global ROTATIONAL_LOADING_SPEED = val
end

"""
    update_position_from_sim!(agent)

Update agent position from RVO simulator
"""
function update_position_from_sim!(agent)
    pt = rvo_get_agent_position(agent)
    @debug "update position from sim agent $(node_id(agent))" pt
    @assert has_parent(agent,agent)
    set_local_transform!(agent,CT.Translation(pt[1],pt[2],0.0))
    if !isapprox(norm(global_transform(agent).translation[1:2] .- pt), 0.0; rtol=1e-6,atol=1e-6) 
        @warn "Agent $node_id(agent) should be at $pt but is at $(global_transform(agent).translation[1:2])"
    end
    return global_transform(agent)
end

"""
    step_environment!(env::PlannerEnv,sim=rvo_global_sim())

Step forward one time step.
"""
function step_environment!(env::PlannerEnv,sim=rvo_global_sim())
    @unpack sched, scene_tree, cache, dt = env
    for v in cache.active_set
        node = get_node(sched,v).node
        cmd = get_cmd(node,env)
        @debug "node: $(node_id(node))" cmd
        # update non-rvo nodes
        apply_cmd!(node,cmd,env)
    end
    # Step RVO
    sim.doStep()
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        tform = update_position_from_sim!(get_node(scene_tree,id))
        @debug "node: $id" tform
    end
    # if necessary, update rvo sim
    active_nodes = [get_node(sched,v) for v in cache.active_set]
    rvo_nodes = filter(rvo_eligible_node, active_nodes)
    if rvo_sim_needs_update(rvo_nodes)
        rvo_set_new_sim!()
        rvo_add_agents!(rvo_nodes)
    end
    return env
end

function TaskGraphs.update_planning_cache!(env::PlannerEnv,time_stamp::Float64)
    @unpack sched, cache = env
    # Skip over nodes that are already planned or just don't need planning
    while true
        done = true
        for v in collect(cache.active_set)
            node = get_node(sched,v)
            if is_goal(node,env)
                TaskGraphs.update_planning_cache!(nothing,sched,cache,v,time_stamp)
                @assert !(v in cache.active_set)
                done = false
            end
        end
        if done
            break
        end
    end
    TaskGraphs.process_schedule!(sched)
    cache
end

# CRCBS.is_goal
CRCBS.is_goal(n::ScheduleNode,env::PlannerEnv) = is_goal(n.node,env)
function CRCBS.is_goal(node::EntityGo,env)
    agent = entity(node)
    state = global_transform(agent).translation
    goal = global_transform(goal_config(node))
    if is_within_capture_distance(state,goal)
        return true
    end
end
"""
    CRCBS.is_goal(node::RobotGo,sched,scene_tree)

If next node is FormTransportUnit, ensure that everybody else is in position.
"""
function CRCBS.is_goal(node::RobotGo,env)
    @unpack sched, scene_tree = env

    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))
    # if norm(goal.translation .- state.translation) >= HG.capture_distance()
    if !is_within_capture_distance(state,goal)
        return false
    end
    if is_terminal_node(sched,node)
        return true
    end
    next_node = get_node(sched,outneighbors(sched,node)[1])
    if matches_template(FormTransportUnit,next_node)
        tu = entity(next_node)
        for (id,tform) in robot_team(tu)
            robot = get_node(scene_tree,id)
            if !is_within_capture_distance(tu,robot)
                return false
            end
        end
    end
    return true
end
function CRCBS.is_goal(node::Union{FormTransportUnit,DepositCargo},env)
    @unpack sched, scene_tree = env.scene_tree
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    state = global_transform(cargo)
    goal = global_transform(cargo_goal_config(node))
    return is_within_capture_distance(state,goal)
end
CRCBS.is_goal(node::ConstructionPredicate,env) = true


# TaskGraphs.update_env!(node,sched,scene_tree,cache,time_stamp)
for T in (:BuildPhasePredicate,:EntityConfigPredicate,:ProjectComplete)
    @eval begin
        function get_cmd(node::$T,env)
            return nothing
        end
        function apply_cmd!(node::$T,cmd::Nothing,env)
            return nothing
            # update_planning_cache!(nothing,sched,cache,get_vtx(sched,node),time_stamp)
        end
    end
end

function get_cmd(node::Union{TransportUnitGo,RobotGo},env)
    @unpack sched, scene_tree, dt = env
    agent = entity(node)
    update_position_from_sim!(agent)
    if is_goal(node,env)
        vel = SVector(0.0,0.0,0.0)
        twist = Twist(SVector(0.0,0.0,0.0),SVector(0.0,0.0,0.0))
    else
        tf_error = relative_transform(global_transform(agent), global_transform(goal_config(node)))
        v_max = get_rvo_max_speed(agent)
        ω_max = default_rotational_loading_speed()
        twist = optimal_twist(tf_error,v_max,ω_max,dt)
    end
    rvo_set_agent_pref_velocity!(node,twist.vel[1:2])
    return twist
end
apply_cmd!(node::Union{TransportUnitGo,RobotGo},cmd,args...) = nothing # ConstructionPredicate.rvo_set_agent_pref_velocity!(node,cmd)

function get_cmd(node::Union{FormTransportUnit,DepositCargo},env)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    # compute velocity (angular and translational) for cargo
    tf_error = relative_transform(global_transform(cargo),global_transform(cargo_goal_config(node)))
    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()
    twist = optimal_twist(tf_error,v_max,ω_max,dt)
end
function apply_cmd!(node::Union{FormTransportUnit,DepositCargo},twist,env)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    tform = integrate_twist(twist,dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
end

"""
    Twist
"""
struct Twist
    vel::SVector{3,Float64}
    ω::SVector{3,Float64}
end

export
    optimal_twist,
    integrate_twist

"""
    optimal_twist(tf_error,v_max,ω_max)

Given a pose error `tf_error`, compute the maximum magnitude `Twist` that 
satisfies the bounds on linear and angular velocity and does not overshoot the
goal pose.
    tf_error = inv(state) ∘ goal # transform error from state to goal 
    i.e., state ∘ tf_error == goal
"""
function optimal_twist(tf_error,v_max,ω_max,dt,ϵ_x=1e-4,ϵ_θ=1e-4)
    # translation error
    dx = tf_error.translation
    if norm(dx) <= ϵ_x
        vel = SVector(0.0,0.0,0.0)
    else
        vel = normalize(dx) * min(v_max, norm(dx)/dt)
    end
    vel = any(isnan,vel) ? SVector(0.0,0.0,0.0) : vel
    # rotation error
    r = RotationVec(tf_error.linear) # rotation vector
    θ = SVector(r.sx,r.sy,r.sz) # convert r to svector
    if norm(θ) <= ϵ_θ
        ω = SVector(0.0,0.0,0.0)
    else
        ω = normalize(θ) * min(ω_max, norm(θ)/dt)
    end
    ω = any(isnan,ω) ? SVector(0.0,0.0,0.0) : ω
    Twist(vel,ω)
end

"""
    apply_twist!(tf,twist,dt)

Integrate `twist::Twist` for `dt` seconds to obtain a rigid transform.
"""
function integrate_twist(twist,dt)
    Δx = twist.vel*dt # translation increment
    ΔR = exp(cross_product_operator(twist.ω)*dt)
    Δ = CT.Translation(Δx) ∘ CT.LinearMap(ΔR)
    return Δ
end