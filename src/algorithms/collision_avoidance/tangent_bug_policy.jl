export TangentBugPolicy

@with_kw mutable struct TangentBugPolicy <: DeconflictStrategy
    name::String="TangentBugPolicy"
    mode::Symbol=:MOVE_TOWARD_GOAL
    vmax::Float64=1.0
    dt::Float64=1/40.0
    proximity_tolerance::Float64=1e-2
    agent_radius::Float64=0.5
    planning_radius::Float64=2 * agent_radius
    detour_horizon::Float64=2 * planning_radius
    buffer::Float64=staging_buffer_radius()
    config::Any=identity_linear_map()
    cmd::Twist=Twist(SVector(0.0, 0.0, 0.0), SVector(0.0, 0.0, 0.0))
end

"""
    perform_twist_deconfliction(t::TangentBugPolicy, env, node)

Performs twist deconfliction for an agent `node` within environment `env` using
the Tangent Bug algorithm.

# Arguments
- `t::TangentBugPolicy`: The Tangent Bug policy deconfliction strategy instance, 
which contains parameters like maximum velocity `vmax`, and planning radius.
- `env`: Simulation environment with current state of agents and obstacles.
- `node`: The agent for which the twist is being calculated.

# Returns
- `Twist`: The computed twist (velocity and angular velocity) for the agent to
follow in order to avoid obstacles and move towards its goal.
"""
function perform_twist_deconfliction(t::TangentBugPolicy, env, node)
    @unpack sched, agent_policies, dt = env
    agent = entity(node)
    goal = global_transform(goal_config(node))
    mode = :not_set
    policy = agent_policies[node_id(agent)].nominal_policy
    pos = project_to_2d(global_transform(agent).translation)
    excluded_ids = Set{AbstractID}()
    if parent_build_step_is_active(node, env) && cargo_ready_for_pickup(node, env)
        parent_build_step = get_parent_build_step(sched, node)
        push!(excluded_ids, node_id(parent_build_step))
    end
    # Determine any circles obstacles, potentially inflated, that might
    # interfere with the direct 
    # path to the goal.
    circles = active_staging_circles(env, excluded_ids)
    # Update policy and get goal
    policy.config = global_transform(agent)
    # Apply the Tangent Bug algorithm to compute an alternative goal point that 
    # avoids the obstacles while still progressing towards the original goal.
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
    # Compute the desired twist that moves agent towards this new goal point 
    # without colliding with the obstacles.
    return compute_twist_from_goal(env, agent, new_goal, dt)
end

"""
    first_intersection_pt(circle,p1,p2)

Return closest point of intersection of `circle` with line from `p1` to `p2`
"""
function first_intersection_pt(circle, p1, p2)
    c = get_center(circle)
    r = get_radius(circle)
    pt = project_point_to_line(c, p1, p2)
    b = norm(pt - c)
    if b > r
        return nothing
    end
    a = sqrt(r^2 - b^2)
    vec = pt - p1
    return p1 + normalize(vec) * (norm(vec) - a)
end

"""
    get_closest_interfering_circle(policy,circles,pos,nominal_goal)

Returns the id and bloated (by policy.agent_radius+buffer) circle closest to pos
"""
function get_closest_interfering_circle(
    policy,
    circles,
    pos,
    nominal_goal;
    return_w_no_buffer = false,
)
    @unpack planning_radius,
    detour_horizon,
    proximity_tolerance,
    buffer,
    agent_radius,
    vmax,
    dt = policy
    dmin = Inf
    id = nothing
    circ = nothing
    pt = nominal_goal
    for (circ_id, c) in circles
        x = get_center(c)
        r = get_radius(c)
        bloated_circle =
            LazySets.Ball2(x, r + agent_radius + buffer * (!return_w_no_buffer))
        if circle_intersects_line(bloated_circle, pos, nominal_goal)
            d = norm(x - pos) - get_radius(bloated_circle)  # penetration
            if d < dmin
                # penetration < 0 => pos is in circle
                dmin = d
                id = circ_id
                circ = bloated_circle
                pt = first_intersection_pt(bloated_circle, pos, nominal_goal)
            end
        end
    end
    return id, circ, pt
end

function set_policy_mode!(policy, circ, pos, nominal_goal, parent_step_active)
    @unpack planning_radius,
    detour_horizon,
    proximity_tolerance,
    buffer,
    agent_radius,
    vmax,
    dt = policy
    dmin = Inf
    c = nothing
    r = nothing
    if !(circ === nothing)
        c = get_center(circ)
        r = get_radius(circ)
        dmin = norm(c - pos) - r
    end
    mode = policy.mode
    if circ === nothing
        mode = :MOVE_TOWARD_GOAL
    else
        if norm(nominal_goal - c) < r  # nominal_goal is in circle
            mode = :WAIT_OUTSIDE
            # If we're in a circle and its not active, we need to get out
            goal_in_circle = norm(nominal_goal - c) < r
            robot_in_circle = norm(pos - c) < r
            if !parent_step_active && goal_in_circle && robot_in_circle
                mode = :EXIT_CIRCLE
            end
        elseif dmin + proximity_tolerance >= 0
            # Not currently in a circle, but on a course for intersection
            if norm(nominal_goal - pos) + agent_radius / 2 < norm(nominal_goal - c)
                # Just keep going toward goal (on the clear side). This statement should never be reached
                mode = :MOVE_TOWARD_GOAL
            elseif dmin < detour_horizon
                # Detour
                if dmin < 2 * proximity_tolerance
                    # This is right at circle, so just turn right to skim
                    mode = :MOVE_ALONG_CIRCLE
                else
                    # Pick a tangent point to shoot for
                    mode = :MOVE_TANGENT
                end
            else
                mode = :MOVE_TOWARD_GOAL
            end
        else
            mode = :EXIT_CIRCLE
        end
    end
    policy.mode = mode
end

function tangent_bug_policy!(policy, circles, pos, nominal_goal, parent_step_active)
    @unpack planning_radius,
    detour_horizon,
    proximity_tolerance,
    buffer,
    agent_radius,
    vmax,
    dt = policy
    c = nothing
    r = nothing
    id = nothing
    dmin = Inf
    id, circ, waypoint = get_closest_interfering_circle(policy, circles, pos, nominal_goal)
    if !(circ === nothing)
        c = get_center(circ)
        r = get_radius(circ)
        dmin = norm(c - pos) - r
    end
    # Select operating mode
    mode = set_policy_mode!(policy, circ, pos, nominal_goal, parent_step_active)
    # Select waypoint
    goal = nominal_goal
    if mode == :WAIT_OUTSIDE
        if norm(nominal_goal - pos) + buffer > r
            dvec = normalize(pos - c) * (r + proximity_tolerance)
            dr = vmax * dt
            dθ = 2 * sin(0.5 * dr / r)
            goal = c + [cos(dθ) -sin(dθ); sin(dθ) cos(dθ)] * dvec
        else
            goal = c + normalize(pos - c) * r
        end
    elseif mode == :MOVE_TOWARD_GOAL
        goal = nominal_goal
    elseif mode == :MOVE_ALONG_CIRCLE
        dvec = normalize(pos - c) * (r + proximity_tolerance)
        # Compute sector to traverse
        dr = vmax * dt
        dθ = 2 * sin(0.5 * dr / r)
        goal = c + [cos(dθ) -sin(dθ); sin(dθ) cos(dθ)] * dvec
    elseif mode == :MOVE_TANGENT
        vec = c - pos  # vector from pos to circle center
        ψ = asin(r / norm(vec))  # yaw angle of right tangent line
        dθ = π / 2 - ψ  # CCW angular offset to tangent point
        dvec = normalize(pos - c) * r
        goal = c + [cos(dθ) -sin(dθ); sin(dθ) cos(dθ)] * dvec
        # If goal causes intersection with another circle, choose waypoint instead
        new_id, _, _ = get_closest_interfering_circle(policy, circles, pos, goal)
        if !(new_id === nothing || new_id == id)
            policy.mode = :MOVE_TOWARD_GOAL
            goal = waypoint
        end
    elseif mode == :EXIT_CIRCLE
        vec = pos - c
        if norm(vec) < 1e-3
            goal = nominal_goal
        else
            goal = c + (r + 2 * proximity_tolerance) * normalize(vec)
        end
    else
        throw(ErrorException("Unknown controller mode $mode"))
    end
    # Compute desired velocity
    vec = goal - pos
    if norm(vec) > vmax * dt
        vel = vmax * normalize(vec)
    else
        vel = vec / dt
    end
    policy.cmd = Twist(SVector(vel[1:2]..., 0.0), SVector(0.0, 0.0, 0.0))
    return goal
end

query_policy_for_goal!(policy::TangentBugPolicy, args...) =
    tangent_bug_policy!(policy, args...)

function update_env_with_deconfliction(t::TangentBugPolicy, scene_tree, env)
    for node in get_nodes(env.sched)
        if matches_template(Union{RobotStart,FormTransportUnit}, node)
            n = entity(node)
            agent_radius = get_radius(get_base_geom(n, HypersphereKey()))
            vmax = get_agent_max_speed(n)
            env.agent_policies[node_id(n)] = ConstructionBots.VelocityController(
                nominal_policy = TangentBugPolicy(
                    dt = env.dt,
                    vmax = vmax,
                    agent_radius = agent_radius,
                ),
                dispersion_policy = nothing,
            )
        end
    end
end