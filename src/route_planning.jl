export PlannerEnv,
    step_environment!,
    get_cmd,
    apply_cmd!,
    project_complete,
    close_node!,
    preprocess_env!,
    update_planning_cache!,
    parent_build_step_is_active,
    update_parent_build_status!


export set_use_deconfliction, use_rvo

function set_use_deconfliction(deconflict_strategies)
    if in(:RVO, deconflict_strategies)
        global USE_RVO = true
    else
        global USE_RVO = false
    end
end

# TODO(tashakim): eliminate reliance on this global variable, and replace
# instances in route planning that depend on this boolean indicator.
# Figure out why USE_RVO needs to be set `true` as default here. This
# shouldn't be the case. 
global USE_RVO = true
use_rvo() = USE_RVO

global STAGING_BUFFER_RADIUS = 0.0
staging_buffer_radius() = STAGING_BUFFER_RADIUS
function set_staging_buffer_radius!(val)
    global STAGING_BUFFER_RADIUS = val
end

export Twist, optimal_twist, integrate_twist

"""
    Twist
"""
struct Twist
    vel::SVector{3,Float64}
    ω::SVector{3,Float64}
end
Base.zero(::Type{Twist}) = Twist(SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0))

"""
    optimal_twist(tf_error,v_max,ω_max)

Given a pose error `tf_error`, compute the maximum magnitude `Twist` that
satisfies the bounds on linear and angular velocity and does not overshoot the
goal pose.
    tf_error = inv(state) ∘ goal # transform error from state to goal
    i.e., state ∘ tf_error == goal
"""
function optimal_twist(tf_error, v_max, ω_max, dt, ϵ_x = 1e-4, ϵ_θ = 1e-4)
    # translation error
    dx = tf_error.translation
    if norm(dx) <= ϵ_x
        vel = SVector(0.0, 0.0, 0.0)
    else
        vel = normalize(dx) * min(v_max, norm(dx) / dt)
    end
    vel = any(isnan, vel) ? SVector(0.0, 0.0, 0.0) : vel
    # rotation error
    r = RotationVec(tf_error.linear) # rotation vector
    θ = SVector(r.sx, r.sy, r.sz) # convert r to svector
    if norm(θ) <= ϵ_θ
        ω = SVector(0.0, 0.0, 0.0)
    else
        ω = normalize(θ) * min(ω_max, norm(θ) / dt)
    end
    ω = any(isnan, ω) ? SVector(0.0, 0.0, 0.0) : ω
    Twist(vel, ω)
end

"""
    apply_twist!(tf,twist,dt)

Integrate `twist::Twist` for `dt` seconds to obtain a rigid transform.
"""
function integrate_twist(twist, dt)
    Δx = twist.vel * dt # translation increment
    ΔR = exp(cross_product_operator(twist.ω) * dt)
    Δ = CoordinateTransformations.Translation(Δx) ∘ CoordinateTransformations.LinearMap(ΔR)
    return Δ
end

"""
    PlannerEnv

Contains the Environment state and definition.
"""
@with_kw struct PlannerEnv
    sched::OperatingSchedule = OperatingSchedule()
    scene_tree::SceneTree = SceneTree()
    cache::PlanningCache = initialize_planning_cache(sched)
    staging_circles::Dict{AbstractID,LazySets.Ball2} = Dict{AbstractID,LazySets.Ball2}()
    active_build_steps::Set{AbstractID} = Set{AbstractID}()
    # TODO(tashakim): Store default time step as fields of RVO state, and 
    # remove the global var from rvo_interface
    dt::Float64 = DEFAULT_TIME_STEP
    agent_policies::Dict = Dict()
    agent_parent_build_step_active::Dict = Dict()
    staging_buffers::Dict{AbstractID,Float64} = Dict{AbstractID,Float64}() # dynamic buffer for staging areas
    max_robot_go_id::Int64 = Inf
    max_cargo_id::Int64 = Inf
    deconflict_strategies::Vector{Symbol} = [:Nothing]
end

node_is_active(env, node) = get_vtx(env.sched, node_id(node)) in env.cache.active_set
node_is_closed(env, node) = get_vtx(env.sched, node_id(node)) in env.cache.closed_set

function update_parent_build_status!(env::PlannerEnv, node)
    env.agent_parent_build_step_active[node_id(entity(node))] =
        parent_build_step_is_active(node, env)
end

function parent_build_step_is_active(id::AbstractID, env::PlannerEnv)
    return env.agent_parent_build_step_active[id]
end

function parent_build_step_is_active(node, env::PlannerEnv)
    build_step = get_parent_build_step(env.sched, node)
    !(build_step === nothing) && node_id(build_step) in env.active_build_steps
end
function cargo_ready_for_pickup(
    n::Union{FormTransportUnit,TransportUnitGo,DepositCargo},
    env::PlannerEnv,
)
    @unpack sched, scene_tree, cache = env
    cargo = get_node(scene_tree, cargo_id(entity(n)))
    if matches_template(ObjectNode, cargo)
        return true
    else
        return node_is_closed(env, AssemblyComplete(cargo))
    end
end
function cargo_ready_for_pickup(n::Union{RobotStart,RobotGo}, env::PlannerEnv)
    if outdegree(env.sched, n) < 1
        return false
    end
    cargo_ready_for_pickup(get_node(env.sched, first(outneighbors(env.sched, n))), env)
end
cargo_ready_for_pickup(n::ScheduleNode, env::PlannerEnv) =
    cargo_ready_for_pickup(n.node, env)

"""
    LOADING_SPEED

The max velocity with which a part may be loaded (e.g., by LiftIntoPlace,
FormTransportUnit,DepositCargo)
"""
global LOADING_SPEED = 0.1
default_loading_speed() = LOADING_SPEED
function set_default_loading_speed!(val::Float64)
    global LOADING_SPEED = val
end

"""
    ROTATIONAL_LOADING_SPEED

The max rotational velocity with which a part may be loaded (e.g., by
LiftIntoPlace,FormTransportUnit,DepositCargo)
"""
global ROTATIONAL_LOADING_SPEED = 0.1
default_rotational_loading_speed() = ROTATIONAL_LOADING_SPEED
function set_default_rotational_loading_speed!(val::Float64)
    global ROTATIONAL_LOADING_SPEED = val
end

function simulate!(env; max_time_steps = 2000)
    @unpack sched, cache = env
    iters = 0
    for k = 1:max_time_steps
        iters = k
        if mod(k, 100) == 0
            @info " ******************* BEGINNING TIME STEP $k: $(length(cache.closed_set))/$(nv(sched)) nodes closed *******************"
        end
        step_environment!(env)
        update_planning_cache!(env, 0.0)
        if project_complete(env)
            println("PROJECT COMPLETE!")
            break
        end
    end
    return project_complete(env), iters
end

function get_active_pos(env::PlannerEnv)
    pos_dict = Dict{Int,Vector{Float64}}()
    for v in env.cache.active_set
        n = get_node(env.sched, v)
        agent = entity(n)
        pos = global_transform(agent).translation
        pos_dict[v] = pos
    end
    return pos_dict
end

"""
    step_environment!(env::PlannerEnv, sim=rvo_global_sim())

Step forward one time step.
"""
# TODO(tashakim): Refactor sim default param and step RVO, line 282
function step_environment!(env::PlannerEnv, sim = rvo_global_sim())

    prev_active_pos_dict = get_active_pos(env)
    for v in env.cache.active_set
        node = get_node(env.sched, v).node
        cmd = get_cmd(node, env)
        apply_cmd!(node, cmd, env) # update non-rvo nodes
    end

    for (_, policy) in env.agent_policies
        if !isnothing(policy.dispersion_policy)
            update_parent_build_status!(env, policy.dispersion_policy.node)
        end
    end

    # Step RVO
    if !isnothing(sim)
        sim.doStep()
    end
    # TODO(tashakim): Refactor rvo_global_id_map
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        tform = update_agent_position_in_sim!(env, get_node(env.scene_tree, id))
    end

    # swap transport unit positions if necessary
    swap_first_paralyzed_transport_unit!(env, prev_active_pos_dict)

    # Set velocities to zero for all agents. The pref velocities are only overwritten if
    # agent is "active" in the next time step
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        set_agent_pref_velocity!(env, get_node(env.scene_tree, id), (0.0, 0.0))
    end
    return env
end

function update_planning_cache!(env::PlannerEnv, time_stamp::Float64)
    @unpack sched, cache = env
    # Skip over nodes that are already planned or just don't need planning
    updated = false
    newly_updated = Set{Int}()
    while true
        done = true
        for v in collect(cache.active_set)
            node = get_node(sched, v)
            if is_goal(node, env)
                close_node!(node, env)
                @info "node $(summary(node_id(node))) finished."
                update_planning_cache!(nothing, sched, cache, v, time_stamp)
                # @info "active nodes $([get_vtx_id(sched,v) for v in cache.active_set])"
                @assert !(v in cache.active_set) && (v in cache.closed_set)
                push!(newly_updated, v)
                done = false
                updated = true
            end
        end
        if done
            break
        end
    end
    if updated
        process_schedule!(sched)
        preprocess_env!(env)
        update_simulation!(env)
    end
    newly_updated
end

"""
    close_node!(node,env)

Ensure that a node is completed
"""
close_node!(node::ScheduleNode, env::PlannerEnv) = close_node!(node.node, env)
close_node!(::ConstructionPredicate, env::PlannerEnv) = nothing #close_node!(n,env)
close_node!(n::OpenBuildStep, env::PlannerEnv) = push!(env.active_build_steps, node_id(n))
function close_node!(node::CloseBuildStep, env::PlannerEnv)
    @unpack sched, scene_tree = env
    assembly = get_assembly(node)
    @info "Closing BuildingStep $(node_id(node))"
    delete!(env.active_build_steps, node_id(OpenBuildStep(node)))
    for (id, tform) in assembly_components(node)
        if !has_edge(scene_tree, assembly, id)
            if !capture_child!(scene_tree, assembly, id)
                @warn "Assembly $(string(node_id(assembly))) is unable to capture child $(string(id)). Current relative transform is $(relative_transform(assembly,get_node(scene_tree,id))), but should be $(child_transform(assembly,id))" assembly id
            end
        end
        @assert has_edge(scene_tree, assembly, id)
    end
end

"""
ensure that all transport units (if active) are formed or (conversely)
disbanded
"""
function preprocess_env!(env::PlannerEnv)
    @unpack sched, scene_tree, cache = env
    for v in cache.active_set
        node = get_node(sched, v)
        if matches_template(FormTransportUnit, node)
            if !capture_robots!(entity(node), scene_tree)
                @warn "Unable to capture robots: $(node_id(node))"
            end
        elseif matches_template(RobotGo, node)
            # ensure that node does not still have a parent
            @assert has_parent(entity(node), entity(node))
        end
    end
end

function project_complete(env::PlannerEnv)
    @unpack sched, cache = env
    for n in get_nodes(sched)
        if matches_template(ProjectComplete, n)
            if !(get_vtx(sched, n) in cache.closed_set)
                return false
            end
        end
    end
    return true
end

is_goal(n::ScheduleNode, env::PlannerEnv) = is_goal(n.node, env)
is_goal(node::ConstructionPredicate, env::PlannerEnv) = true
function is_goal(node::EntityGo, env::PlannerEnv)
    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))
    return is_within_capture_distance(state, goal)
end
"""
    is_goal(node::RobotGo,sched,scene_tree)

If next node is FormTransportUnit, ensure that everybody else is in position.
"""
function is_goal(node::Union{RobotGo,TransportUnitGo}, env::PlannerEnv)
    @unpack sched, scene_tree, cache = env
    agent = entity(node)
    state = global_transform(agent)
    goal = global_transform(goal_config(node))

    if !is_within_capture_distance(state, goal)
        return false
    end
    if is_terminal_node(sched, node)
        # Does this need to be modified?
        # return true
        return false
    end
    next_node = get_node(sched, outneighbors(sched, node)[1])
    # Cannot reach goal until next_node is ready to become active
    # Should take care of requiring the parent build step to be active
    for v in inneighbors(sched, next_node)
        if !((v in cache.active_set) || (v in cache.closed_set))
            return false
        end
    end
    # Cannot reach goal until all robots are in place
    if matches_template(FormTransportUnit, next_node)
        tu = entity(next_node)
        for (id, tform) in robot_team(tu)
            robot = get_node(scene_tree, id)
            if !is_within_capture_distance(tu, robot)
                return false
            end
        end
    end
    return true
end
function is_goal(node::Union{FormTransportUnit,DepositCargo}, env::PlannerEnv)
    agent = entity(node)
    cargo = get_node(env.scene_tree, cargo_id(agent))
    state = global_transform(cargo)
    goal = global_transform(cargo_goal_config(node))
    return is_within_capture_distance(state, goal)
end
function is_goal(node::LiftIntoPlace, env::PlannerEnv)
    cargo = entity(node)
    state = global_transform(cargo)
    goal = global_transform(goal_config(node))
    return is_within_capture_distance(state, goal)
end

function swap_first_paralyzed_transport_unit!(
    env::PlannerEnv,
    pos_before_step::Dict{Int,Vector{Float64}},
)
    @unpack sched, scene_tree, cache, dt = env
    for v in cache.active_set
        n = get_node(sched, v)
        if matches_template(RobotGo, n) && outdegree(sched, n) >= 1
            next_node = get_node(sched, first(outneighbors(sched, n)))
            while outdegree(sched, next_node) >= 1 && matches_template(RobotGo, next_node)
                next_node = get_node(sched, first(outneighbors(sched, next_node)))
            end
            if matches_template(FormTransportUnit, next_node)
                # is robot stuck?
                agent = entity(n)
                transport_unit = entity(next_node)
                if has_parent(agent, transport_unit) ||
                   is_within_capture_distance(transport_unit, agent)
                    continue
                end
                circ = get_cached_geom(transport_unit, HypersphereKey())
                ctr = get_center(circ)[1:2]
                rad = get_radius(circ)
                vel = get_agent_pref_velocity(env, n)
                pos = global_transform(agent).translation[1:2]
                agent_radius = get_radius(get_cached_geom(agent, HypersphereKey()))
                if norm(pos .- ctr) < agent_radius + rad # within circle
                    swap = false
                    if norm([vel...]) < 1e-6 # stuck
                        swap = true
                    elseif haskey(pos_before_step, v)
                        pos_bs = pos_before_step[v]
                        vel_est = norm((pos[1:2] - pos_bs[1:2])) / dt
                        if vel_est < 1e-2
                            swap = true
                        end
                    end
                    if swap
                        swap_carrying_positions!(next_node.node, n.node, env)
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
function find_best_swap_candidate(
    node::FormTransportUnit,
    agent_node::RobotGo,
    env::PlannerEnv,
)
    @unpack sched, scene_tree = env
    transport_unit = entity(node)
    agent = entity(agent_node)
    goal = global_transform(goal_config(agent_node))
    state = global_transform(agent)
    if is_within_capture_distance(transport_unit, agent)
        # no need to swap
        return nothing
    end
    # find best swap
    closest_id = nothing
    dist = Inf
    agent_dist = norm(goal.translation .- state.translation)
    for (id, tform) in robot_team(transport_unit)
        if !(id == node_id(agent))
            other_agent = get_node(scene_tree, id)
            other_state = global_transform(other_agent)
            if is_within_capture_distance(transport_unit, other_agent)
                d1 = norm(goal.translation .- other_state.translation)
                if d1 > agent_dist
                    continue
                end
                d = norm(state.translation .- other_state.translation)
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
function swap_carrying_positions!(
    node::FormTransportUnit,
    agent_node::RobotGo,
    env::PlannerEnv,
)
    @unpack sched, scene_tree = env
    other_id = find_best_swap_candidate(node, agent_node, env)
    if !(other_id === nothing)
        @assert matches_template(RobotID, other_id)
        agent = entity(agent_node)
        other_agent = get_node(scene_tree, other_id)
        swap_positions!(agent, other_agent)
        # swap positions in rvo_sim as well
        # TODO(tashakim): Get position from node instead of directly from 
        # rvo_get_agent_position (and below)
        if use_rvo()
            tmp = rvo_get_agent_position(agent)
            rvo_set_agent_position!(agent, rvo_get_agent_position(other_agent))
            rvo_set_agent_position!(other_agent, tmp)
        end
    end
    return agent_node
end

"""
    swap_positions!(agent1,agent2)

Swap positions of two robots in simulation.
"""
function swap_positions!(agent1, agent2)
    @info "Swapping agent $(summary(node_id(agent1))) with $(summary(node_id(agent2)))"
    tmp = global_transform(agent1)
    set_desired_global_transform!(agent1, global_transform(agent2))
    set_desired_global_transform!(agent2, tmp)
    return agent1, agent2
end

function query_policy_for_goal! end

include("deconfliction_interface.jl")
include("tangent_bug.jl")
include("potential_fields.jl")

"""
    circle_avoidance_policy()

Returns a 2D goal vector that will take the robot outside of circular boundary
regions while pursuing its main goal
"""
function circle_avoidance_policy(
    circles,
    agent_radius,
    pos,
    nominal_goal;
    planning_radius::Float64 = agent_radius * 2,
    detour_horizon::Float64 = 2 * planning_radius,
    buffer = staging_buffer_radius(),
)
    dmin = Inf
    id = nothing
    circ = nothing
    # Get the first circle to be intersected
    # for (i,c) in enumerate(circles)
    for (circ_id, c) in circles
        x = get_center(c)
        r = get_radius(c)
        bloated_circle = LazySets.(x, r + agent_radius + buffer)
        if circle_intersects_line(bloated_circle, pos, nominal_goal)
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
    c = get_center(circ)
    r = get_radius(circ)
    if norm(nominal_goal - c) < r # nominal_goal is in circle
        # wait outside
        # how to scale buffer here?
        if norm(nominal_goal - c) > 1e-3
            goal = c + normalize(nominal_goal - c) * r
        else
            goal = c + normalize(pos - c) * r
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
                goal = pos .+ [-dvec[2], dvec[1]]
            else
                # Pick a tangent point to shoot for
                pts = nearest_points_between_circles(pos[1:2], c[1:2], norm(pos - c), r)
                goal = sort([pts...], by = p -> norm(nominal_goal - [p...]))[1]
            end
        else
            goal = nominal_goal
        end
    else
        # get goal points on the edge of the circle
        pts = nearest_points_between_circles(
            pos[1:2],
            get_center(circ)[1:2],
            planning_radius,
            get_radius(circ),
        )
        if pts === nothing
            goal = nominal_goal
        else
            # select closest point to goal
            goal = sort([pts...], by = p -> norm(nominal_goal - [p...]))[1]
        end
    end
    nominal_pt =
        pos + normalize(nominal_goal - pos) * min(planning_radius, norm(nominal_goal - pos))
    f = g -> circle_intersection_with_line(circ, pos, g)
    # if norm(nominal_pt .- c) > norm(goal .- c)
    if f(nominal_pt) > f(goal)
        return nominal_goal
    else
        return goal
    end
end

inflate_circle(circ::LazySets.Ball2, r::Float64) =
    LazySets.Ball2(get_center(circ), get_radius(circ) + r)
# inflate_circle(circ::GeometryBasics.HyperSphere,r::Float64) = GeometryBasics.HyperSphere(get_center(circ),get_radius(circ)+r)

function active_staging_circles(env, exclude_ids = Set())
    buffer = env.staging_buffers # to increase radius of staging circles when necessary
    node_iter = (
        get_node(env.sched, id).node for
        id in env.active_build_steps if !(id in exclude_ids)
    )
    circle_iter = (
        node_id(n) => project_to_2d(
            inflate_circle(get_cached_geom(n.staging_circle), get(buffer, node_id(n), 0.0)),
        ) for n in node_iter
    )
end

function inflate_staging_circle_buffers!(
    env,
    policy,
    agent,
    circle_ids;
    threshold = 0.2,
    delta = 0.1 * default_robot_radius(),
    delta_max = 4 * default_robot_radius(),
)
    @unpack staging_buffers, dt = env
    # desird change in position
    desired_dx = dt * project_to_2d(policy.cmd.vel)
    prev_pos = project_to_2d(policy.config.translation)
    pos = project_to_2d(global_transform(agent).translation)
    # true change in position
    dx = pos - prev_pos
    if norm(dx) < threshold * norm(desired_dx)
        # increase buffer
        for id in circle_ids
            buffer = get(staging_buffers, id, 0.0) + delta
            if buffer < delta_max - delta
                buffer = min(buffer, delta_max)
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
function active_build_step_countdown(step, env::PlannerEnv)
    @unpack sched = env
    open_step = get_node(sched, OpenBuildStep(step)).node
    k = 0
    while !(node_id(open_step) in env.active_build_steps)
        k += 1
        close_step = get_node(sched, first(inneighbors(sched, open_step))).node
        if matches_template(CloseBuildStep, close_step)
            open_step = get_node(sched, OpenBuildStep(close_step)).node
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
function get_twist_cmd(node, env::PlannerEnv)
    @unpack sched, scene_tree, agent_policies, cache, dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    mode = :not_set
    ############ TangentBugPolicy #############
    policy = agent_policies[node_id(agent)].nominal_policy
    if !(policy === nothing)
        pos = project_to_2d(global_transform(agent).translation)
        excluded_ids = Set{AbstractID}()

        #? Can we use the cached version here?
        if parent_build_step_is_active(node, env) && cargo_ready_for_pickup(node, env)
            parent_build_step = get_parent_build_step(sched, node)
            push!(excluded_ids, node_id(parent_build_step))
        end
        # get circle obstacles, potentially inflated
        circles = active_staging_circles(env, excluded_ids)

        # update policy and get goal
        policy.config = global_transform(agent)

        # TangentBug Policy
        parent_step_is_active = parent_build_step_is_active(node, env)
        goal_pt = query_policy_for_goal!(
            policy,
            circles,
            pos,
            project_to_2d(goal.translation),
            parent_step_is_active,
        )
        new_goal =
            CoordinateTransformations.Translation(goal_pt..., 0.0) ∘
            CoordinateTransformations.LinearMap(goal.linear)

        _, circ, _ = get_closest_interfering_circle(
            policy,
            circles,
            pos,
            project_to_2d(goal.translation);
            return_w_no_buffer = true,
        )
        mode = set_policy_mode!(
            policy,
            circ,
            pos,
            project_to_2d(goal.translation),
            parent_step_is_active,
        )

        twist = compute_twist_from_goal(env, agent, new_goal, dt) # nominal twist
    else
        twist = compute_twist_from_goal(env, agent, goal, dt)
    end
    if use_rvo()
        ############ Hacky Traffic Thinning #############
        # set nominal velocity to zero if close to goal (HACK)
        parent_step = get_parent_build_step(sched, node)
        if !(parent_step === nothing)
            countdown = active_build_step_countdown(parent_step.node, env)
            dist_to_goal = norm(goal.translation .- global_transform(agent).translation)

            unit_radius = get_base_geom(entity(node), HypersphereKey()).radius
            if (mode != :EXIT_CIRCLE) && (dist_to_goal < 15 * unit_radius)
                # So the agent doesn't crowd its destination
                if matches_template(TransportUnitGo, node) && countdown >= 1
                    twist = Twist(0.0 * twist.vel, twist.ω)
                elseif matches_template(RobotGo, node) && countdown >= 3
                    twist = Twist(0.0 * twist.vel, twist.ω)
                end
            end
        end
        ############ Potential Field Policy #############
        policy = agent_policies[node_id(agent)].dispersion_policy
        # For RobotGo node, ensure that parent assembly is "pickup-able"
        ready_for_pickup = cargo_ready_for_pickup(node, env)
        #? Can we use the cached version here?
        build_step_active = parent_build_step_is_active(node, env)
        if !(policy === nothing)
            update_dist_to_nearest_active_agent!(policy, env)
            update_buffer_radius!(policy, node, build_step_active, ready_for_pickup)
            if !(build_step_active && ready_for_pickup)
                # update policy
                policy.node = node
                # compute target position
                nominal_twist = twist
                pos = project_to_2d(global_transform(agent).translation)
                va = nominal_twist.vel[1:2]
                target_pos = pos .+ va * dt
                # commanded velocity from current position
                vb = -1.0 * compute_potential_gradient!(policy, env, pos)
                # commanded velocity from current position
                vc = -1.0 * compute_potential_gradient!(policy, env, target_pos)
                # blend the three velocities
                a = 1.0
                b = 1.0
                c = 0.0
                v = (a * va + b * vb + c * vc)
                vel = clip_velocity(v, policy.vmax)
                # compute goal
                goal_pt = pos + vel * dt
                goal =
                    CoordinateTransformations.Translation(goal_pt..., 0.0) ∘
                    CoordinateTransformations.LinearMap(goal.linear)
                twist = compute_twist_from_goal(env, agent, goal, dt) # nominal twist
            else
                !(policy === nothing)
                policy.dist_to_nearest_active_agent = 0.0
            end
        end
    end
    # return goal
    return twist
end

function compute_twist_from_goal(
    env,
    agent,
    goal,
    dt;
    v_max = get_agent_max_speed(agent),
    ω_max = default_rotational_loading_speed(),
)
    tf_error = relative_transform(global_transform(agent), goal)
    return optimal_twist(tf_error, v_max, ω_max, dt)
end

get_cmd(
    ::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},
    env::PlannerEnv,
) = nothing
function get_cmd(node::Union{TransportUnitGo,RobotGo}, env::PlannerEnv)
    agent = entity(node)
    update_agent_position_in_sim!(env, agent)
    set_agent_priority!(env, node)

    #? Do we want to set the max speed to zero because its at its goal? I think we should
    #? still have agents be able to move, albeit slower? 10% of normal max speed?
    if is_goal(node, env)
        twist = zero(Twist) # desired velocity it to stay still #? Could change this?
        max_speed = 0.1 * get_agent_max_speed(agent)
    else
        twist = get_twist_cmd(node, env)
        twist_vel = norm(twist.vel[1:2])
        if twist_vel > 0.0
            max_speed = min(get_agent_max_speed(agent), twist_vel)
        else
            max_speed = get_agent_max_speed(agent)
        end
    end

    set_agent_max_speed!(env, node, max_speed)
    set_agent_pref_velocity!(env, node, twist.vel[1:2])
    return twist
end
function get_cmd(node::Union{FormTransportUnit,DepositCargo}, env::PlannerEnv)
    agent = entity(node)
    cargo = get_node(env.scene_tree, cargo_id(agent))
    # compute velocity (angular and translational) for cargo
    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()
    g_tform = global_transform(cargo_goal_config(node))
    return compute_twist_from_goal(
        env,
        cargo,
        g_tform,
        env.dt,
        v_max = v_max,
        ω_max = ω_max,
    )
end
function get_cmd(node::LiftIntoPlace, env::PlannerEnv)
    cargo = entity(node)
    # compute velocity (angular and translational) for cargo
    t_des = global_transform(goal_config(node))
    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()
    twist = compute_twist_from_goal(env, cargo, t_des, env.dt, v_max = v_max, ω_max = ω_max)
    if norm(twist.ω) >= 1e-2
        # rotate first
        return Twist(0 * twist.vel, twist.ω)
    end
    return twist
end

apply_cmd!(
    ::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},
    cmd,
    env::PlannerEnv,
) = nothing
apply_cmd!(node::CloseBuildStep, cmd::Nothing, env::PlannerEnv) = close_node!(node, env)
apply_cmd!(node::OpenBuildStep, cmd::Nothing, env::PlannerEnv) = close_node!(node, env)
function apply_cmd!(node::FormTransportUnit, twist::Twist, env::PlannerEnv)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree, cargo_id(agent))
    for (robot_id, _) in robot_team(agent)
        if !has_edge(scene_tree, agent, robot_id)
            capture_child!(scene_tree, agent, robot_id)
        end
        @assert has_edge(scene_tree, agent, robot_id)
    end
    tform = integrate_twist(twist, dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
    if is_within_capture_distance(agent, cargo)
        capture_child!(scene_tree, agent, cargo)
        set_agent_max_speed!(env, agent, get_agent_max_speed(agent))
    else
        set_agent_max_speed!(env, agent, 0.0)
    end
end
function apply_cmd!(node::DepositCargo, twist::Twist, env::PlannerEnv)
    @unpack sched, scene_tree, cache, dt = env
    agent = entity(node)
    cargo = get_node(scene_tree, cargo_id(agent))
    tform = integrate_twist(twist, dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
    if is_goal(node, env)
        disband!(scene_tree, agent)
        set_agent_max_speed!(env, agent, get_agent_max_speed(agent))
    else
        set_agent_max_speed!(env, agent, 0.0)
    end
end
function apply_cmd!(node::LiftIntoPlace, twist::Twist, env::PlannerEnv)
    @unpack sched, scene_tree, cache, dt = env
    cargo = entity(node)
    tform = integrate_twist(twist, dt)
    set_local_transform!(cargo, local_transform(cargo) ∘ tform)
end
function apply_cmd!(node::Union{TransportUnitGo,RobotGo}, twist::Twist, env::PlannerEnv)
    @unpack sched, scene_tree, cache, dt = env
    if !use_rvo()
        agent = entity(node)
        tform = integrate_twist(twist, dt)
        set_local_transform!(agent, local_transform(agent) ∘ tform)
    end
end
