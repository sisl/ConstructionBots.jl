export
    PlannerEnv,
    step_environment!,
    get_cmd,
    apply_cmd!,
    project_complete


export
    use_rvo,
    set_use_rvo!,
    avoid_staging_areas,
    set_avoid_staging_areas!

global USE_RVO = true
use_rvo() = USE_RVO
function set_use_rvo!(val)
    global USE_RVO = val
end
global AVOID_STAGING_AREAS = false
avoid_staging_areas() = AVOID_STAGING_AREAS
function set_avoid_staging_areas!(val)
    global AVOID_STAGING_AREAS = val
end
global STAGING_BUFFER_RADIUS = 0.0
staging_buffer_radius() = STAGING_BUFFER_RADIUS
function set_staging_buffer_radius!(val)
    global STAGING_BUFFER_RADIUS = val
end

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
    Δ = CoordinateTransformations.Translation(Δx) ∘ CoordinateTransformations.LinearMap(ΔR)
    return Δ
end

"""
    PlannerEnv

Contains the Environment state and definition.
"""
@with_kw struct PlannerEnv
    sched::OperatingSchedule    = OperatingSchedule()
    scene_tree::SceneTree       = SceneTree()
    cache::PlanningCache        = initialize_planning_cache(sched)
    staging_circles::Dict{AbstractID,Ball2} = Dict{AbstractID,Ball2}()
    active_build_steps::Set{AbstractID} = Set{AbstractID}()
    # active_assemblies::Set{AbstractID} = Set{AbstractID}() # taken care of by active build step
    dt::Float64                 = rvo_default_time_step()
    agent_policies::Dict        = Dict()
    staging_buffers::Dict{AbstractID,Float64} = Dict{AbstractID,Float64}() # dynamic buffer for staging areas
end

node_is_active(env,node) = get_vtx(env.sched,node_id(node)) in env.cache.active_set
node_is_closed(env,node) = get_vtx(env.sched,node_id(node)) in env.cache.closed_set

function parent_build_step_is_active(node,env)
    build_step = get_parent_build_step(env.sched,node)
    !(build_step === nothing) && node_id(build_step) in env.active_build_steps
end
function cargo_ready_for_pickup(n::Union{FormTransportUnit,TransportUnitGo,DepositCargo},env)
    @unpack sched, scene_tree, cache = env
    cargo = get_node(scene_tree,cargo_id(entity(n)))
    if matches_template(ObjectNode,cargo)
        return true
    else
        return node_is_closed(env,AssemblyComplete(cargo))
    end
end
function cargo_ready_for_pickup(n::Union{RobotStart,RobotGo},env)
    if outdegree(env.sched,n) < 1
        return false
    end
    cargo_ready_for_pickup(get_node(env.sched,first(outneighbors(env.sched,n))),env)
end
cargo_ready_for_pickup(n::ScheduleNode,env) = cargo_ready_for_pickup(n.node,env)

"""
    set_rvo_priority!(env,node)

Low alpha means higher priority
"""
function set_rvo_priority!(env,node)
    @unpack sched, scene_tree, cache, staging_circles = env
    if matches_template(Union{FormTransportUnit,DepositCargo},node)
        alpha = 0.0
    elseif parent_build_step_is_active(node,env)
        if cargo_ready_for_pickup(node,env)
            if matches_template(TransportUnitGo,node)
                alpha = 0.0
            else
                alpha = 0.1
            end
        else
            alpha = 0.5
        end
    else
        alpha = 1.0
    end
    rvo_set_agent_alpha!(node,alpha)
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
    dt_vis = env.dt,
    )
    @unpack sched, scene_tree, cache, dt = env
    t0 = 0.0
    time_stamp = t0
    iters = 0
    for k in 1:max_time_steps
        iters = k
        if mod(k,100) == 0
            @info " ******************* BEGINNING TIME STEP $k: $(length(cache.closed_set))/$(nv(sched)) nodes closed *******************"
        end
        step_environment!(env)
        newly_updated = TaskGraphs.update_planning_cache!(env,0.0)
        update_visualizer_function(env,newly_updated)
        sleep(dt_vis)
        time_stamp = t0+k*dt
        if project_complete(env)
            println("PROJECT COMPLETE!")
            break
        end
    end
    return project_complete(env), iters
end

"""
    update_position_from_sim!(agent)

Update agent position from RVO simulator
"""
function update_position_from_sim!(agent)
    pt = rvo_get_agent_position(agent)
    # @debug "update position from sim agent $(node_id(agent))" pt
    @assert has_parent(agent,agent) "agent $(node_id(agent)) should be its own parent"
    set_local_transform!(agent,CoordinateTransformations.Translation(pt[1],pt[2],0.0))
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
        if use_rvo()
            # @info "Updating position from rvo"
            tform = update_position_from_sim!(get_node(scene_tree,id))
        end
    end
    # swap transport unit positions if necessary
    swap_first_paralyzed_transport_unit!(env::PlannerEnv)
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        # Set velocities to zero for any agents that are no longer "active"
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
        for node in rvo_nodes
            set_rvo_priority!(env,node)
        end
    end
end

function TaskGraphs.update_planning_cache!(env::PlannerEnv,time_stamp::Float64)
    @unpack sched, cache = env
    # Skip over nodes that are already planned or just don't need planning
    updated = false
    newly_updated = Set{Int}()
    while true
        done = true
        for v in collect(cache.active_set)
            node = get_node(sched,v)
            if CRCBS.is_goal(node,env)
                close_node!(node,env)
                @info "node $(summary(node_id(node))) finished."
                TaskGraphs.update_planning_cache!(nothing,sched,cache,v,time_stamp)
                # @info "active nodes $([get_vtx_id(sched,v) for v in cache.active_set])"
                @assert !(v in cache.active_set) && (v in cache.closed_set)
                push!(newly_updated,v)
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
    newly_updated
end

"""
    close_node!(node,env)

Ensure that a node is completed
"""
close_node!(node::ScheduleNode,env) = close_node!(node.node,env)
close_node!(::ConstructionPredicate,env) =  nothing #close_node!(n,env)
close_node!(n::OpenBuildStep,env) = push!(env.active_build_steps,node_id(n))
function close_node!(node::CloseBuildStep,env)
    @unpack sched, scene_tree = env
    assembly = get_assembly(node)
    @info "Closing BuildingStep $(node_id(node))"
    delete!(env.active_build_steps,node_id(OpenBuildStep(node)))
    for (id,tform) in assembly_components(node)
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
    # done = true
    for n in get_nodes(sched)
        if matches_template(ProjectComplete,n)
            if !(get_vtx(sched,n) in cache.closed_set)
                return false
            end
        end
    end
    return true
    # if isempty(cache.active_set)
    #     @assert nv(sched) == length(cache.closed_set)
    #     return true
    # end
    # return false
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
# function CRCBS.is_goal(node::RobotGo,env)
function CRCBS.is_goal(node::Union{RobotGo,TransportUnitGo},env)
    @unpack sched, scene_tree, cache = env
    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))
    if !is_within_capture_distance(state,goal)
        return false
    end
    if is_terminal_node(sched,node)
        # Does this need to be modified?
        # return true
        return false
    end
    next_node = get_node(sched,outneighbors(sched,node)[1])
    # Cannot reach goal until next_node is ready to become active
    # Should take care of requiring the parent build step to be active
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

# function avoid_active_staging_areas(node::Union{TransportUnitGo,RobotGo},twist,env,ϵ=1e-3)
#     @unpack sched, scene_tree, dt = env
#     agent = entity(node)
#     r = HierarchicalGeometry.get_radius(get_base_geom(agent,HypersphereKey()))
#     pos = HierarchicalGeometry.project_to_2d(global_transform(agent).translation)
#     goal = HierarchicalGeometry.project_to_2d(global_transform(goal_config(node)).translation)
#     parent_build_step = get_parent_build_step(sched,node)
#     dmin = Inf
#     id = nothing
#     circ = nothing
#     for build_step_id in env.active_build_steps
#         build_step_id == node_id(parent_build_step) ? continue : nothing
#         build_step = get_node(sched,build_step_id).node
#         circle = HierarchicalGeometry.project_to_2d(get_cached_geom(build_step.staging_circle))
#         bloated_circle = Ball2(HierarchicalGeometry.get_center(circle),HierarchicalGeometry.get_radius(circle)+r)
#         if HierarchicalGeometry.circle_intersects_line(bloated_circle,pos,goal)
#             d = HierarchicalGeometry.get_radius(bloated_circle) - norm(HierarchicalGeometry.get_center(bloated_circle) - pos) # penetration
#             if d < 0 && d < dmin
#                 # in circle
#                 dmin = d
#                 id = build_step_id
#                 circ = bloated_circle
#             end
#         end
#     end
#     # If on the "exiting" end of circle, just leave,
#     if id === nothing ||  norm(goal - pos) + ϵ < norm(goal - HierarchicalGeometry.get_center(circ))
#         return twist
#     end
#     dvec = pos-HierarchicalGeometry.get_center(circ)
#     # return a vector tangent CCW to the circle
#     if norm(dvec) < HierarchicalGeometry.get_radius(circ) + ϵ
#         # start position is on or in circ
#         vec = SVector(-dvec[2],dvec[1])
#     else
#         # start position is not on or in circ
#         pt_left, pt_right = HierarchicalGeometry.get_tangent_pts_on_circle(circ,pos)
#         vec = pt_right-pos
#     end
#     vel = norm(twist.vel) * normalize(vec)
#     @info "$(summary(node_id(agent))) avoiding $id with vel = $vel"
#     Twist(SVector(vel..., 0.0), twist.ω)
# end

function swap_first_paralyzed_transport_unit!(env::PlannerEnv)
    @unpack sched, scene_tree, cache = env
    for v in cache.active_set
        n = get_node(sched,v)
        if matches_template(RobotGo,n) && outdegree(sched,n) >= 1
            next_node = get_node(sched,first(outneighbors(sched,n)))
            while outdegree(sched,next_node) >= 1 && matches_template(RobotGo,next_node)
                next_node = get_node(sched,first(outneighbors(sched,next_node)))
            end
            if matches_template(FormTransportUnit,next_node)
                # is robot stuck?
                agent = entity(n)
                transport_unit = entity(next_node)
                if has_parent(agent,transport_unit) || is_within_capture_distance(transport_unit,agent)
                    continue
                end
                circ = get_cached_geom(transport_unit,HypersphereKey())
                ctr = HierarchicalGeometry.get_center(circ)[1:2]
                rad = HierarchicalGeometry.get_radius(circ)
                vel = rvo_get_agent_pref_velocity(agent)
                pos = global_transform(agent).translation[1:2]
                agent_radius = HierarchicalGeometry.get_radius(get_cached_geom(agent,HypersphereKey()))
                if norm(pos .- ctr) < agent_radius + rad # within circle
                    if norm([vel...]) < 1e-6 # stuck
                        # @info "Swapping agent $(summary(node_id(agent))) with $(summary(node_id(agent)))"
                        swap_carrying_positions!(
                            next_node.node,
                            n.node,
                            env)
                    end
                end
            end
        end
    end
end

"""
    find_best_swap_candidate(node::FormTransportUnit,agent_node::RobotGo,env)

For agent
"""
function find_best_swap_candidate(node::FormTransportUnit,agent_node::RobotGo,env)
    @unpack sched, scene_tree = env
    transport_unit = entity(node)
    agent = entity(agent_node)
    goal = global_transform(goal_config(agent_node))
    state = global_transform(agent)
    if is_within_capture_distance(transport_unit,agent)
        # no need to swap
        return nothing
    end
    # find best swap
    closest_id = nothing
    dist = Inf # norm(goal.translation .- state.translation)
    agent_dist = norm(goal.translation .- state.translation)
    for (id,tform) in robot_team(transport_unit)
        if !(id == node_id(agent))
            other_agent = get_node(scene_tree,id)
            other_state = global_transform(other_agent)
            if is_within_capture_distance(transport_unit,other_agent)
                d1 = norm(goal.translation .- other_state.translation)
                if d1 > agent_dist
                    continue
                end
                d = norm(state.translation .- other_state.translation)
                # d2 = norm(state.translation .- other_state.translation)
                # d = d1+d2
                if d < dist
                    dist = d
                    closest_id = id
                end
            end
        end
    end
    return closest_id
end

"""
    swap_carrying_positions!(node::FormTransportUnit,agent::RobotNode,env)

To be executed if `agent` is within the hypersphere but stuck.
"""
function swap_carrying_positions!(node::FormTransportUnit,agent_node::RobotGo,env)
    @unpack sched, scene_tree = env
    other_id = find_best_swap_candidate(node,agent_node,env)
    if !(other_id === nothing)
        @assert matches_template(RobotID,other_id)
        agent = entity(agent_node)
        other_agent = get_node(scene_tree,other_id)
        swap_positions!(agent,other_agent)
        # swap positions in rvo_sim as well
        tmp = rvo_get_agent_position(agent)
        rvo_set_agent_position!(agent,rvo_get_agent_position(other_agent))
        rvo_set_agent_position!(other_agent,tmp)
    end
    return agent_node
end

"""
    swap_positions!(agent1,agent2)

Swap positions of two robots in simulation.
"""
function swap_positions!(agent1,agent2)
    @warn "Swapping agent $(summary(node_id(agent1))) with $(summary(node_id(agent2)))"
    tmp = global_transform(agent1)
    set_desired_global_transform!(agent1,global_transform(agent2))
    set_desired_global_transform!(agent2,tmp)
    return agent1, agent2
end

function query_policy_for_goal! end

include("tangent_bug.jl")
include("potential_fields.jl")

"""
    circle_avoidance_policy()

Returns a 2D goal vector that will take the robot outside of circular boundary
regions while pursuing its main goal
"""
function circle_avoidance_policy(circles,agent_radius,pos,nominal_goal;
        planning_radius::Float64=agent_radius*2,
        detour_horizon::Float64=2*planning_radius,
        buffer=staging_buffer_radius(),
        )
    dmin = Inf
    id = nothing
    circ = nothing
    # Get the first circle to be intersected
    # for (i,c) in enumerate(circles)
    for (circ_id,c) in circles
        x = HierarchicalGeometry.get_center(c)
        r = HierarchicalGeometry.get_radius(c)
        bloated_circle = Ball2(x,r+agent_radius+buffer)
        if HierarchicalGeometry.circle_intersects_line(bloated_circle,pos,nominal_goal)
            d = norm(x - pos) - (r + agent_radius) #- norm(x - pos) # penetration
            if d < dmin
                # penetration < 0 => pos is in circle
                dmin = d
                # idx = i
                id = circ_id
                circ = bloated_circle
            end
        end
    end
    goal = nominal_goal
    if circ === nothing
        # nothing in the way
        return nominal_goal
    end
    c = HierarchicalGeometry.get_center(circ)
    r = HierarchicalGeometry.get_radius(circ)
    if norm(nominal_goal - c) < r # nominal_goal is in circle
        # wait outside
        # how to scale buffer here?
        if norm(nominal_goal - c) > 1e-3
            goal = c + normalize(nominal_goal - c)*r
        else
            goal = c + normalize(pos - c)*r
        end
    elseif dmin >= 0
        # not currently in a circle, but on a course for intersection
        if norm(nominal_goal - pos) < norm(nominal_goal - c)
            # Just keep going toward goal (on the clear side)--this statement should never be reached
            goal = nominal_goal
        elseif dmin < detour_horizon
            # detour to "skim" the circle
            if dmin < 1e-3
                dvec = pos - c
                goal = pos .+ [-dvec[2],dvec[1]]
            else
                # Pick a tangent point to shoot for
                pts = GraphUtils.nearest_points_between_circles(
                    pos[1:2], c[1:2], norm(pos - c), r
                )
                goal = sort([pts...],by=p->norm(nominal_goal-[p...]))[1]
            end
        else
            goal = nominal_goal
        end
    else
        # get goal points on the edge of the circle
        pts = GraphUtils.nearest_points_between_circles(
            pos[1:2],HierarchicalGeometry.get_center(circ)[1:2],planning_radius,HierarchicalGeometry.get_radius(circ),
        )
        if pts === nothing
            goal = nominal_goal
        else
            # select closest point to goal
            goal = sort([pts...],by=p->norm(nominal_goal-[p...]))[1]
        end
    end
    nominal_pt = pos + normalize(nominal_goal - pos) * min(planning_radius,norm(nominal_goal - pos))
    f = g->HierarchicalGeometry.circle_intersection_with_line(circ,pos,g)
    # if norm(nominal_pt .- c) > norm(goal .- c)
    if f(nominal_pt) > f(goal)
        return nominal_goal
    else
        return goal
    end
end

inflate_circle(circ::Ball2,r::Float64) = Ball2(HierarchicalGeometry.get_center(circ),HierarchicalGeometry.get_radius(circ)+r)
# inflate_circle(circ::GeometryBasics.HyperSphere,r::Float64) = GeometryBasics.HyperSphere(HierarchicalGeometry.get_center(circ),HierarchicalGeometry.get_radius(circ)+r)

function active_staging_circles(env,exclude_ids=Set())
    buffer = env.staging_buffers # to increase radius of staging circles when necessary
    node_iter = (get_node(env.sched,id).node for id in env.active_build_steps if !(id in exclude_ids))
    circle_iter = (node_id(n)=>HierarchicalGeometry.project_to_2d(
        inflate_circle(get_cached_geom(n.staging_circle), get(buffer,node_id(n),0.0)
        )) for n in node_iter)
end

function inflate_staging_circle_buffers!(env,policy,agent,circle_ids;
        threshold=0.2,
        delta = 0.1*default_robot_radius(),
        delta_max = 4*default_robot_radius(),
    )
    @unpack staging_buffers, dt = env
    # desird change in position
    desired_dx     = dt * HierarchicalGeometry.project_to_2d(policy.cmd.vel)
    prev_pos       = HierarchicalGeometry.project_to_2d(policy.config.translation)
    pos            = HierarchicalGeometry.project_to_2d(global_transform(agent).translation)
    # true change in position
    dx             = pos - prev_pos
    if norm(dx) < threshold * norm(desired_dx)
        # increase buffer
        for id in circle_ids
            buffer = get(staging_buffers,id,0.0) + delta
            if buffer < delta_max - delta
                buffer = min(buffer,delta_max)
                staging_buffers[id] = buffer
                @info "increasing radius buffer of $(summary(id)) to $(buffer) because $(summary(node_id(agent))) is stuck"
            end
        end
    end
end

@with_kw mutable struct VelocityController
    nominal_policy = nothing # TangentBugPolicy
    dispersion_policy = nothing # potential field
    # RVO policy
end

"""
    active_build_step_countdown(step,env)

Measures how many build steps a step is away from becoming active.
"""
function active_build_step_countdown(step,env)
    @unpack sched = env
    open_step = get_node(sched,OpenBuildStep(step)).node
    k = 0
    while !(node_id(open_step) in env.active_build_steps)
        k += 1
        close_step = get_node(sched,first(inneighbors(sched,open_step))).node
        if matches_template(CloseBuildStep,close_step)
            open_step = get_node(sched,OpenBuildStep(close_step)).node
        else
            break
        end
    end
    return k
end

"""
    get_twist_cmd(node,env)

Query the agent's policy to get the desired twist
"""
function get_twist_cmd(node,env)
    @unpack sched, scene_tree, agent_policies, cache, dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    ############ TangentBugPolicy #############
    policy = agent_policies[node_id(agent)].nominal_policy
    if !(policy === nothing)
        pos = HierarchicalGeometry.project_to_2d(global_transform(agent).translation)
        r = HierarchicalGeometry.get_radius(get_base_geom(agent,HypersphereKey()))
        excluded_ids = Set{AbstractID}()
        if parent_build_step_is_active(node,env)
            parent_build_step = get_parent_build_step(sched,node)
            push!(excluded_ids, node_id(parent_build_step))
        end
        # get circle obstacles, potentially inflated
        circles = active_staging_circles(env,excluded_ids)
        # INFLATE CIRCLES IF NECESSARY
        # if policy.mode == :MOVE_TOWARD_GOAL
        #     inflate_staging_circle_buffers!(env,policy,agent,excluded_ids)
        # end
        # update policy and get goal
        policy.config = global_transform(agent)
        goal_pt = query_policy_for_goal!(policy,circles,pos,HierarchicalGeometry.project_to_2d(goal.translation))
        new_goal = CoordinateTransformations.Translation(goal_pt...,0.0) ∘ CoordinateTransformations.LinearMap(goal.linear)
        twist = compute_twist_from_goal(agent,new_goal,dt) # nominal twist
        # @info "nominal vel: $(twist.vel)"
    else
        twist = compute_twist_from_goal(agent,goal,dt)
    end
    if use_rvo()
        ############ Hacky Traffic Thinning #############
        # set nominal velocity to zero if close to goal (HACK)
        parent_step = get_parent_build_step(sched,node)
        if !(parent_step === nothing)
            countdown = active_build_step_countdown(parent_step.node,env)
            dist_to_goal = norm(goal.translation .- global_transform(agent).translation)
            if dist_to_goal < 20*default_robot_radius()
                # So the agent doesn't crowd its destination
                if matches_template(TransportUnitGo,node) && countdown >= 1
                    twist = Twist(0.0*twist.vel,twist.ω)
                elseif matches_template(RobotGo,node) && countdown >= 3
                    twist = Twist(0.0*twist.vel,twist.ω)
                end
            end
        end
        ############ Potential Field Policy #############
        policy = agent_policies[node_id(agent)].dispersion_policy
        # For RobotGo node, ensure that parent assembly is "pickup-able"
        ready_for_pickup = cargo_ready_for_pickup(node,env)
        build_step_active = parent_build_step_is_active(node,env)
        if !(policy === nothing || (build_step_active && ready_for_pickup))
            # update policy
            policy.node = node
            update_dist_to_nearest_active_agent!(policy)
            update_buffer_radius!(policy)
            # compute target position
            nominal_twist = twist
            pos = HierarchicalGeometry.project_to_2d(global_transform(agent).translation)
            va = nominal_twist.vel[1:2]
            target_pos = pos .+ va * dt
            # commanded velocity from current position
            # vb = compute_velocity_command!(policy,pos)
            vb = -1.0 * compute_potential_gradient!(policy,pos)
            # commanded velocity from current position
            # vc = compute_velocity_command!(policy,target_pos)
            vc = -1.0 * compute_potential_gradient!(policy,target_pos)
            # blend the three velocities
            a = 1.0
            b = 1.0
            c = 0.0
            # v = (a*va+b*vb+c*vc) / (a+b+c)
            v = (a*va+b*vb+c*vc)
            vel = clip_velocity(v,policy.vmax)
            # compute goal
            goal_pt = pos + vel*dt
            goal = CoordinateTransformations.Translation(goal_pt...,0.0) ∘ CoordinateTransformations.LinearMap(goal.linear)
            twist = compute_twist_from_goal(agent,goal,dt) # nominal twist
        elseif !(policy === nothing)
            policy.dist_to_nearest_active_agent = 0.0
        end
    end
    # return goal
    return twist
end

function compute_twist_from_goal(agent,goal,dt)
    tf_error = relative_transform(global_transform(agent), goal)
    v_max = get_rvo_max_speed(agent)
    ω_max = default_rotational_loading_speed()
    twist = optimal_twist(tf_error,v_max,ω_max,dt)
end
get_cmd(::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},env) = nothing
function get_cmd(node::Union{TransportUnitGo,RobotGo},env)
    @unpack sched, scene_tree, dt = env
    agent = entity(node)
    if use_rvo()
        update_position_from_sim!(agent)
    end
    if is_goal(node,env) && !is_terminal_node(sched,node)
        twist = zero(Twist)
        max_speed = 0.0
    else
        twist = get_twist_cmd(node,env)
        # goal = get_twist_cmd(node,env)
        # tf_error = relative_transform(global_transform(agent), goal)
        # v_max = get_rvo_max_speed(agent)
        # ω_max = default_rotational_loading_speed()
        # twist = optimal_twist(tf_error,v_max,ω_max,dt)
        max_speed = get_rvo_max_speed(agent)
    end
    if isa(node,TransportUnitGo)
        set_rvo_priority!(env,node)
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
    if norm(twist.ω) >= 1e-2
        # rotate first
        return Twist(0*twist.vel,twist.ω)
    end
    # @info "twist for LiftIntoPlace" twist
    twist
end

apply_cmd!(::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},cmd,env) = nothing
apply_cmd!(node::CloseBuildStep,cmd::Nothing,env) = close_node!(node,env)
apply_cmd!(node::OpenBuildStep,cmd::Nothing,env) = close_node!(node,env)
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
function apply_cmd!(node::Union{TransportUnitGo,RobotGo},twist,env)
    @unpack sched, scene_tree, cache, dt = env
    if !use_rvo()
        agent = entity(node)
        tform = integrate_twist(twist,dt)
        set_local_transform!(agent, local_transform(agent) ∘ tform)
    end
end
