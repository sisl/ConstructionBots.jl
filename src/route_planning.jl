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

global STAGING_BUFFER_RADIUS = 0.0
staging_buffer_radius() = STAGING_BUFFER_RADIUS

function set_staging_buffer_radius!(val)
    global STAGING_BUFFER_RADIUS = val
end

export Twist, optimal_twist, integrate_twist

"""
    struct Twist

Represents a twist in 3D space, encapsulating both linear velocity (`vel`) and 
angular velocity (`ω`), each represented as a 3D static vector.
"""
struct Twist
    vel::SVector{3,Float64}
    ω::SVector{3,Float64}
end

Base.zero(::Type{Twist}) = Twist(SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0))

"""
    optimal_twist(tf_error, v_max, ω_max)

Given a pose error `tf_error`, compute the maximum magnitude `Twist` that
satisfies the bounds on linear and angular velocity and does not overshoot the
goal pose.
    tf_error = inv(state) ∘ goal # transform error from state to goal
    i.e., state ∘ tf_error == goal
"""
function optimal_twist(tf_error, v_max, ω_max, dt, ϵ_x = 1e-4, ϵ_θ = 1e-4)
    dx = tf_error.translation  # translation error
    if norm(dx) <= ϵ_x
        vel = SVector(0.0, 0.0, 0.0)
    else
        vel = normalize(dx) * min(v_max, norm(dx) / dt)
    end
    vel = any(isnan, vel) ? SVector(0.0, 0.0, 0.0) : vel  # rotation error
    r = RotationVec(tf_error.linear)  # rotation vector
    θ = SVector(r.sx, r.sy, r.sz)  # convert r to svector
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
    Δx = twist.vel * dt  # translation increment
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
    staging_buffers::Dict{AbstractID,Float64} = Dict{AbstractID,Float64}()  # dynamic buffer for staging areas
    max_robot_go_id::Int64 = Inf
    max_cargo_id::Int64 = Inf
    deconfliction_type = ReciprocalVelocityObstacle()
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
step_environment!(env::PlannerEnv, sim=rvo_global_sim()) -> PlannerEnv

Advances the simulation environment by one time step. It integrates the current
twists of all agents, updates the planning cache, and potentially modifies agent
positions based on the chosen deconfliction strategy.

- `env`: The planning environment to be stepped.
- `sim`: (Optional) The simulation object, typically an RVO simulator.

Returns the updated `PlannerEnv`.
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
    # TODO(tashakim): Fold below methods into common update method for all non-
    # RVO deconfliction strategies. Below currently only apply to RVO.
    # Step RVO
    if !isnothing(sim)
        sim.doStep()
    end
    # TODO(tashakim): Refactor rvo_global_id_map
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        tform = update_agent_position_in_sim!(
            env.deconfliction_type,
            env,
            get_node(env.scene_tree, id),
        )
    end
    # Swap transport unit positions if necessary
    swap_first_paralyzed_transport_unit!(env, prev_active_pos_dict)
    # Set velocities to zero for all agents. The pref velocities are only overwritten if
    # agent is "active" in the next time step
    for id in get_vtx_ids(ConstructionBots.rvo_global_id_map())
        set_agent_pref_velocity!(
            env.deconfliction_type,
            get_node(env.scene_tree, id),
            (0.0, 0.0),
        )
    end
    return env
end

"""
update_planning_cache!(env::PlannerEnv, time_stamp::Float64) -> Set{Int}

Updates the planning cache of the environment `env` based on the current 
operational state and the given `time_stamp`. It marks nodes as active or closed 
as appropriate and triggers any necessary state transitions.

- `env`: The planning environment whose cache is to be updated.
- `time_stamp`: The current time stamp, used for time-dependent operations.

Returns a set of node IDs that were newly updated during this operation.
"""
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
        update_simulation!(env.deconfliction_type, env)
    end
    newly_updated
end

"""
    close_node!(node, env)

Marks a node as completed within the planning environment `env`. Depending on 
the node type, this might involve updating the active build steps, adjusting 
the scene tree, or other cleanup operations.

- `node`: The node to be marked as completed.
- `env`: The planning environment.
"""
close_node!(node::ScheduleNode, env::PlannerEnv) = close_node!(node.node, env)
close_node!(::ConstructionPredicate, env::PlannerEnv) = nothing  # close_node!(n,env)
close_node!(n::OpenBuildStep, env::PlannerEnv) = push!(env.active_build_steps, node_id(n))
function close_node!(node::CloseBuildStep, env::PlannerEnv)
    @unpack sched, scene_tree = env
    assembly = get_assembly(node)
    @info "Closing BuildingStep $(node_id(node))"
    delete!(env.active_build_steps, node_id(OpenBuildStep(node)))
    for (id, tform) in assembly_components(node)
        if !has_edge(scene_tree, assembly, id)
            if !capture_child!(scene_tree, assembly, id)
                @warn "Assembly $(string(node_id(assembly))) is unable to 
                capture child $(string(id)). Current relative transform is 
                $(relative_transform(assembly,get_node(scene_tree,id))), 
                but should be $(child_transform(assembly,id))" assembly id
            end
        end
        @assert has_edge(scene_tree, assembly, id)
    end
end

"""
Ensure all transport units (if active) are formed or (conversely) disbanded.
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
            # Ensure that node does not still have a parent
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
        # TODO: Check if this needs to be modified
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

"""
swap_first_paralyzed_transport_unit!(env::PlannerEnv, pos_before_step::Dict{Int, 
Vector{Float64}})

Attempts to swap the position of the first paralyzed transport unit with another
unit within the planning environment `env`. A transport unit is considered 
paralyzed if it has not moved significantly from its position before the 
current simulation step, as determined by comparing `pos_before_step` with the 
current position. This function is intended to help resolve deadlock situations
by swapping the positions of units.

- `env`: The planning environment containing the transport units and states.
- `pos_before_step`: A dictionary mapping transport unit IDs to their positions
 before the current simulation step.
"""
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
                # Check if robot is stuck
                agent = entity(n)
                transport_unit = entity(next_node)
                if has_parent(agent, transport_unit) ||
                   is_within_capture_distance(transport_unit, agent)
                    continue
                end
                circ = get_cached_geom(transport_unit, HypersphereKey())
                ctr = get_center(circ)[1:2]
                rad = get_radius(circ)
                vel = get_agent_pref_velocity(env.deconfliction_type, n)
                pos = global_transform(agent).translation[1:2]
                agent_radius = get_radius(get_cached_geom(agent, HypersphereKey()))
                if norm(pos .- ctr) < agent_radius + rad  # within circle
                    swap = false
                    if norm([vel...]) < 1e-6  # stuck
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
    find_best_swap_candidate(node::FormTransportUnit, agent_node::RobotGo, 
    env::PlannerEnv) -> Union{Nothing, Int}

Identifies the best swap candidate for a given robot associated with a 
transport unit, aiming to resolve situations where the robot is stuck or unable 
to progress towards its goal.

- `node`: Transport unit node in question.
- `agent_node`: Robot node that might need to be swapped, due to being stuck or 
paralyzed.
- `env`: The planning environment.
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
        return nothing
    end
    # Find best swap
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
    swap_carrying_positions!(node::FormTransportUnit, agent::RobotNode, env)

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
        swap_positions!(env.deconfliction_type, agent, other_agent)
    end
    return agent_node
end

function query_policy_for_goal! end

include("algorithms/collision_avoidance/deconfliction_strategy.jl")

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
    for (circ_id, c) in circles
        x = get_center(c)
        r = get_radius(c)
        bloated_circle = LazySets.(x, r + agent_radius + buffer)
        if circle_intersects_line(bloated_circle, pos, nominal_goal)
            d = norm(x - pos) - (r + agent_radius)  #- norm(x - pos)  # penetration
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
        # Nothing is in the way
        return nominal_goal
    end
    c = get_center(circ)
    r = get_radius(circ)
    if norm(nominal_goal - c) < r  # nominal_goal is in circle
        # Wait outside
        # TODO: Figure out how to scale buffer here
        if norm(nominal_goal - c) > 1e-3
            goal = c + normalize(nominal_goal - c) * r
        else
            goal = c + normalize(pos - c) * r
        end
    elseif dmin >= 0
        # Not currently in a circle, but on a course for intersection
        if norm(nominal_goal - pos) < norm(nominal_goal - c)
            # Keep going toward goal (on the clear side). This statement should never be reached
            goal = nominal_goal
        elseif dmin < detour_horizon
            # Detour to "skim" the circle
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
        # Get goal points on the edge of the circle
        pts = nearest_points_between_circles(
            pos[1:2],
            get_center(circ)[1:2],
            planning_radius,
            get_radius(circ),
        )
        if pts === nothing
            goal = nominal_goal
        else
            # Select closest point to goal
            goal = sort([pts...], by = p -> norm(nominal_goal - [p...]))[1]
        end
    end
    nominal_pt =
        pos + normalize(nominal_goal - pos) * min(planning_radius, norm(nominal_goal - pos))
    f = g -> circle_intersection_with_line(circ, pos, g)
    if f(nominal_pt) > f(goal)
        return nominal_goal
    else
        return goal
    end
end

inflate_circle(circ::LazySets.Ball2, r::Float64) =
    LazySets.Ball2(get_center(circ), get_radius(circ) + r)

"""
    active_staging_circles(env, exclude_ids=Set()) -> Dict

Generates a dictionary of active staging circles within the planning 
environment, optionally excluding specified IDs. Each staging circle is
represented by a circle object with a center position and radius, adjusted by 
dynamic buffers where applicable.

- `env`: The planning environment, containing all relevant state information.
- `exclude_ids`: (Optional) A set of IDs to exclude from the returned dictionary.
"""
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

"""
    inflate_staging_circle_buffers!(env, policy, agent, circle_ids; 
    threshold=0.2, delta=0.1 * default_robot_radius(), 
    delta_max=4 * default_robot_radius())

Adjusts the radius of staging circles based on agent movement to prevent 
deadlock situations. If an agent moves less than a specified threshold, the 
function increases the buffer radius of associated staging circles to provide 
more space for maneuvering.

- `env`: The planning environment.
- `policy`: The current policy dictating agent movement.
- `agent`: The agent for which the staging circles are considered.
- `circle_ids`: IDs of the circles to potentially adjust.
- `threshold`: Movement threshold below which buffers are increased.
- `delta`: The amount by which to increase the staging circle buffer.
- `delta_max`: The maximum allowable buffer size.
"""
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
    # Desired change in position
    desired_dx = dt * project_to_2d(policy.cmd.vel)
    prev_pos = project_to_2d(policy.config.translation)
    pos = project_to_2d(global_transform(agent).translation)
    # True change in position
    dx = pos - prev_pos
    if norm(dx) < threshold * norm(desired_dx)
        # Increase buffer
        for id in circle_ids
            buffer = get(staging_buffers, id, 0.0) + delta
            if buffer < delta_max - delta
                buffer = min(buffer, delta_max)
                staging_buffers[id] = buffer
                @info "increasing radius buffer of $(summary(id)) to $(buffer) 
                because $(summary(node_id(agent))) is stuck"
            end
        end
    end
end

@with_kw mutable struct VelocityController
    nominal_policy = nothing  # TangentBugPolicy
    dispersion_policy = nothing  # PotentialFields
    # TODO: Add RVO policy if using VelocityController
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
    compute_twist_from_goal(env, agent, goal, dt; 
    v_max=get_agent_max_speed(agent), 
    ω_max=default_rotational_loading_speed()) -> Twist

Computes the twist required for an agent to progress towards a goal 
configuration from its current state within the specified time step `dt`, 
subject to maximum linear and angular velocity constraints `v_max` and `ω_max`.

- `env`: The planning environment.
- `agent`: The agent for which the twist is being computed.
- `goal`: The target configuration towards which the agent is trying to move.
- `dt`: The time step over which the twist should be applied. This parameter
    influences how aggressively the agent tries to reach its goal.
- `v_max` (optional): The maximum linear velocity that agent can achieve.
- `ω_max` (optional): The maximum angular velocity that agent can achieve.
"""
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

"""
    get_cmd(node, env::PlannerEnv) -> Twist

Retrieves the next command (as a `Twist`) for a given node based on the current
state of the planning environment. The command is determined by considering the 
node's goal, any relevant deconfliction strategies and environmental constraints.

- `node`: The node (e.g., agent, robot) for which to retrieve the command.
- `env`: The planning environment.
"""
get_cmd(
    ::Union{BuildPhasePredicate,EntityConfigPredicate,ProjectComplete},
    env::PlannerEnv,
) = nothing

function get_cmd(node::Union{TransportUnitGo,RobotGo}, env::PlannerEnv)
    agent = entity(node)
    update_agent_position_in_sim!(env.deconfliction_type, env, agent)
    set_agent_priority!(env.deconfliction_type, env, node)
    # TODO: Figure out if we want to set the max speed to zero because its at
    # its goal. We may still want agents to be able to move, albeit slower 
    # e.g. 10% of normal max speed
    if is_goal(node, env)
        twist = zero(Twist) # desired velocity it to stay still #? Could change this?
        max_speed = 0.1 * get_agent_max_speed(agent)
    else
        twist = perform_twist_deconfliction(env.deconfliction_type, env, node)
        twist_vel = norm(twist.vel[1:2])
        if twist_vel > 0.0
            max_speed = min(get_agent_max_speed(agent), twist_vel)
        else
            max_speed = get_agent_max_speed(agent)
        end
    end
    set_agent_max_speed!(env.deconfliction_type, node, max_speed)
    set_agent_pref_velocity!(env.deconfliction_type, node, twist.vel[1:2])
    return twist
end

function get_cmd(node::Union{FormTransportUnit,DepositCargo}, env::PlannerEnv)
    agent = entity(node)
    cargo = get_node(env.scene_tree, cargo_id(agent))
    # Compute velocity (angular and translational) for cargo
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
    # Compute velocity (angular and translational) for cargo
    t_des = global_transform(goal_config(node))
    v_max = default_loading_speed()
    ω_max = default_rotational_loading_speed()
    twist = compute_twist_from_goal(env, cargo, t_des, env.dt, v_max = v_max, ω_max = ω_max)
    if norm(twist.ω) >= 1e-2
        # Rotate first
        return Twist(0 * twist.vel, twist.ω)
    end
    return twist
end

"""
    apply_cmd!(node, cmd::Twist, env::PlannerEnv)

Applies a given command to a node, updating its state within the planning 
environment. This function integrates the provided `Twist` (linear and angular 
velocities) over the environment's time step, adjusting the node's position or 
other relevant attributes.

- `node`: The node (e.g., agent, robot) to which the command is applied.
- `cmd`: The command to apply, encapsulated as a `Twist`.
- `env`: The planning environment.
"""
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
        set_agent_max_speed!(env.deconfliction_type, agent, get_agent_max_speed(agent))
    else
        set_agent_max_speed!(env.deconfliction_type, agent, 0.0)
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
        set_agent_max_speed!(env.deconfliction_type, agent, get_agent_max_speed(agent))
    else
        set_agent_max_speed!(env.deconfliction_type, agent, 0.0)
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
    if !(env.deconfliction_type isa ReciprocalVelocityObstacle)
        agent = entity(node)
        tform = integrate_twist(twist, dt)
        set_local_transform!(agent, local_transform(agent) ∘ tform)
    end
end
