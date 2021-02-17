export 
    PlannerEnv,
    step_environment!,
    get_cmd,
    apply_cmd!,
    project_complete

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

function simulate!(env,update_visualizer_function;
    max_time_steps=2000,
    )
    @unpack sched, scene_tree, cache, dt = env
    t0 = 0.0
    time_stamp = t0
    # configs = TransformDict{AbstractID}(node_id(node)=>global_transform(node) for node in get_nodes(scene_tree))
    for k in 1:max_time_steps
        step_environment!(env)
        # debugging
        # for node in get_nodes(scene_tree)
        #     if norm(global_transform(node).translation - configs[node_id(node)].translation) >= 0.5
        #         @warn "node $(node_id(node)) just jumped" env.cache.active_set
        #     end
        #     configs[node_id(node)] = global_transform(node)
        # end
        TaskGraphs.update_planning_cache!(env,0.0)
        if project_complete(env)
            println("PROJECT COMPLETE!")
            break
        end
        update_visualizer_function(env)
        sleep(dt)
        time_stamp = t0+k*dt
    end
    return env
end


"""
    update_position_from_sim!(agent)

Update agent position from RVO simulator
"""
function update_position_from_sim!(agent)
    pt = rvo_get_agent_position(agent)
    # @debug "update position from sim agent $(node_id(agent))" pt
    @assert has_parent(agent,agent) "agent $(node_id(agent)) should be its own parent"
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
        # update non-rvo nodes
        apply_cmd!(node,cmd,env)
    end
    # Step RVO
    sim.doStep()
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        tform = update_position_from_sim!(get_node(scene_tree,id))
        # Set velocities to zero for any agents that are no longer "active"
        # TODO set preferred velocity to "disperse" velocity
        rvo_set_agent_pref_velocity!(id,(0.0,0.0))
    end
    return env
end

function update_rvo_sim!(env::PlannerEnv)
    @unpack sched, scene_tree, cache = env
    active_nodes = [get_node(sched,v) for v in cache.active_set]
    rvo_nodes = filter(rvo_eligible_node, active_nodes)
    if rvo_sim_needs_update(scene_tree,rvo_nodes)
        @info "New RVO simulation"
        rvo_set_new_sim!()
        rvo_add_agents!(scene_tree,rvo_nodes)
    end
end

function TaskGraphs.update_planning_cache!(env::PlannerEnv,time_stamp::Float64)
    @unpack sched, cache = env
    # Skip over nodes that are already planned or just don't need planning
    updated = false
    while true
        done = true
        for v in collect(cache.active_set)
            node = get_node(sched,v)
            if CRCBS.is_goal(node,env)
                close_node!(node,env)
                @info "node $(node_id(node)) finished."
                TaskGraphs.update_planning_cache!(nothing,sched,cache,v,time_stamp)
                # @info "active nodes $([get_vtx_id(sched,v) for v in cache.active_set])"
                @assert !(v in cache.active_set)
                done = false
                updated = true
            end
        end
        if done
            break
        end
    end
    if updated
        TaskGraphs.process_schedule!(sched)
        preprocess_env!(env)
        update_rvo_sim!(env)
    end
    cache
end

"""
    close_node!(node,env)

Ensure that a node is completed
"""
close_node!(node::ScheduleNode,env) = close_node!(node.node,env)
close_node!(::ConstructionPredicate,env) = nothing
function close_node!(node::CloseBuildStep,env)
    @unpack sched, scene_tree = env
    assembly = get_assembly(node)
    @info "Closing BuildingStep" node
    for (id,tform) in assembly_components(assembly)
        if !has_edge(scene_tree,assembly,id)
            if !capture_child!(scene_tree,assembly,id)
                @warn "Assembly $(string(node_id(assembly))) is unable to capture child $(string(id)). Current relative transform is $(relative_transform(assembly,get_node(scene_tree,id))), but should be $(child_transform(assembly,id))" assembly id 
            end
        end
        @assert has_edge(scene_tree,assembly,id)
    end
end

"""
ensure that all transport units (if active) are formed or (conversely) 
disbanded
"""
function preprocess_env!(env::PlannerEnv)
    @unpack sched, scene_tree, cache = env
    for v in cache.active_set
        node = get_node(sched,v)
        if matches_template(FormTransportUnit,node)
            if !capture_robots!(entity(node),scene_tree)
                @warn "Unable to capture robots: $(node_id(node))"
            end
        elseif matches_template(RobotGo,node)
            # ensure that node does not still have a parent
            @assert has_parent(entity(node),entity(node))
        end
    end
end

function project_complete(env::PlannerEnv)
    @unpack sched, cache = env
    if isempty(cache.active_set)
        @assert nv(sched) == length(cache.closed_set)
        return true
    end
    return false
end

# CRCBS.is_goal
CRCBS.is_goal(n::ScheduleNode,env::PlannerEnv) = is_goal(n.node,env)
CRCBS.is_goal(node::ConstructionPredicate,env) = true
function CRCBS.is_goal(node::EntityGo,env)
    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))
    return is_within_capture_distance(state,goal)
end
"""
    CRCBS.is_goal(node::RobotGo,sched,scene_tree)

If next node is FormTransportUnit, ensure that everybody else is in position.
"""
function CRCBS.is_goal(node::RobotGo,env)
    @unpack sched, scene_tree, cache = env
    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))
    if !is_within_capture_distance(state,goal)
        return false
    end
    if is_terminal_node(sched,node)
        return true
    end
    next_node = get_node(sched,outneighbors(sched,node)[1])
    # Cannot reach goal until next_node is ready to become active
    for v in inneighbors(sched,next_node)
        if !((v in cache.active_set) || (v in cache.closed_set))
            return false
        end
    end
    # Cannot reach goal until all robots are in place 
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
    @unpack sched, scene_tree = env
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    state = global_transform(cargo)
    goal = global_transform(cargo_goal_config(node))
    return is_within_capture_distance(state,goal)
end
function CRCBS.is_goal(node::LiftIntoPlace,env)
    @unpack sched, scene_tree, cache, dt = env
    cargo = entity(node)
    state = global_transform(cargo)
    goal = global_transform(goal_config(node))
    return is_within_capture_distance(state,goal)
end

get_cmd(::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},env) = nothing
function get_cmd(node::Union{TransportUnitGo,RobotGo},env)
    @unpack sched, scene_tree, dt = env
    agent = entity(node)
    update_position_from_sim!(agent)
    if is_goal(node,env) && !is_terminal_node(sched,node)
        twist = zero(Twist)
        max_speed = 0.0
    else
        tf_error = relative_transform(global_transform(agent), global_transform(goal_config(node)))
        v_max = get_rvo_max_speed(agent)
        ω_max = default_rotational_loading_speed()
        twist = optimal_twist(tf_error,v_max,ω_max,dt)
        max_speed = get_rvo_max_speed(agent)
    end
    rvo_set_agent_max_speed!(agent,max_speed)
    rvo_set_agent_pref_velocity!(agent,twist.vel[1:2])
    return twist
end
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
function get_cmd(node::LiftIntoPlace,env)
    @unpack sched, scene_tree, cache, dt = env
    cargo = entity(node)
    # compute velocity (angular and translational) for cargo
    tf_error = relative_transform(global_transform(cargo),global_transform(goal_config(node)))
    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()
    twist = optimal_twist(tf_error,v_max,ω_max,dt)
    # @info "twist for LiftIntoPlace" twist
    twist
end

apply_cmd!(::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},cmd,env) = nothing
function apply_cmd!(node::CloseBuildStep,cmd::Nothing,env)
    close_node!(node,env)
    # @unpack sched, scene_tree = env
    # assembly = get_assembly(node)
    # @info "Closing BuildingStep"
    # for (id,tform) in assembly_components(assembly)
    #     if !has_edge(scene_tree,assembly,id)
    #         capture_child!(scene_tree,assembly,id)
    #     end
    #     @assert has_edge(scene_tree,assembly,id)
    # end
end
function apply_cmd!(node::FormTransportUnit,twist,env)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    for (robot_id,_) in robot_team(agent)
        if !has_edge(scene_tree,agent,robot_id)
            capture_child!(scene_tree,agent,robot_id)
        end
        @assert has_edge(scene_tree,agent,robot_id)
    end
    tform = integrate_twist(twist,dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
    if is_within_capture_distance(agent,cargo)
        capture_child!(scene_tree,agent,cargo)
        rvo_set_agent_max_speed!(agent,get_rvo_max_speed(agent))
    else
        rvo_set_agent_max_speed!(agent,0.0)
    end
end
function apply_cmd!(node::DepositCargo,twist,env)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree,cargo_id(agent))
    tform = integrate_twist(twist,dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
    if is_goal(node,env)
        disband!(scene_tree,agent)
        rvo_set_agent_max_speed!(agent,get_rvo_max_speed(agent))
    else
        rvo_set_agent_max_speed!(agent,0.0)
    end
end
function apply_cmd!(node::LiftIntoPlace,twist,env)
    @unpack sched, scene_tree, cache, dt = env
    cargo = entity(node)
    tform = integrate_twist(twist,dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
end
apply_cmd!(node::Union{TransportUnitGo,RobotGo},cmd,args...) = nothing # ConstructionPredicate.rvo_set_agent_pref_velocity!(node,cmd)


export
    Twist,
    optimal_twist,
    integrate_twist

"""
    Twist
"""
struct Twist
    vel::SVector{3,Float64}
    ω::SVector{3,Float64}
end
Base.zero(::Type{Twist}) = Twist(SVector(0.0,0.0,0.0),SVector(0.0,0.0,0.0))

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