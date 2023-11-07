export TangentBugPolicy

@with_kw mutable struct TangentBugPolicy
    mode                = :MOVE_TOWARD_GOAL
    vmax                = 1.0
    dt                  = 0.05
    proximity_tolerance = 1e-2
    agent_radius        = 0.5
    planning_radius     = 2*agent_radius
    detour_horizon      = 2*planning_radius
    buffer              = staging_buffer_radius()
    # store config and command
    config              = identity_linear_map()
    cmd                 = Twist(SVector(0.0,0.0,0.0),SVector(0.0,0.0,0.0))
end

"""
    first_intersection_pt(circle,p1,p2)

Return closest point of intersection of `circle` with line from `p1` to `p2`
"""
function first_intersection_pt(circle,p1,p2)
    c = get_center(circle)
    r = get_radius(circle)
    pt = project_point_to_line(c,p1,p2)
    b = norm(pt-c)
    if b > r
        return nothing
    end
    a = sqrt(r^2 - b^2)
    vec = pt-p1
    return p1 + normalize(vec) * (norm(vec)-a)
end

"""
    get_closest_interfering_circle(policy,circles,pos,nominal_goal)

Returns the id and bloated (by policy.agent_radius+buffer) circle closest to pos
"""
function get_closest_interfering_circle(policy,circles,pos,nominal_goal; return_w_no_buffer=false)
    @unpack planning_radius, detour_horizon, proximity_tolerance, buffer,
        agent_radius, vmax, dt = policy
    dmin = Inf
    id = nothing
    circ = nothing
    pt = nominal_goal
    for (circ_id,c) in circles
        x = get_center(c)
        r = get_radius(c)
        bloated_circle = LazySets.Ball2(x,r+agent_radius+buffer)
        if circle_intersects_line(bloated_circle,pos,nominal_goal)
            d = norm(x - pos) - get_radius(bloated_circle) # penetration
            if d < dmin
                # penetration < 0 => pos is in circle
                dmin = d
                id = circ_id
                if return_w_no_buffer
                    circ = c
                else
                    circ = bloated_circle
                end
                pt = first_intersection_pt(bloated_circle,pos,nominal_goal)
            end
        end
    end
    return id, circ, pt
end

function set_policy_mode!(policy,circ,pos,nominal_goal, parent_step_active)
    @unpack planning_radius, detour_horizon, proximity_tolerance, buffer,
        agent_radius, vmax, dt = policy
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
        if norm(nominal_goal - c) < r # nominal_goal is in circle
            mode = :WAIT_OUTSIDE

            # If we're in a circle and its not active, we need to get out
            goal_in_circle = norm(nominal_goal - c) < r
            robot_in_circle = norm(pos - c) < r
            if !parent_step_active && goal_in_circle && robot_in_circle
                mode = :EXIT_CIRCLE
            end

        elseif dmin + proximity_tolerance >= 0
            # not currently in a circle, but on a course for intersection
            # if norm(nominal_goal - pos) + 30*proximity_tolerance < norm(nominal_goal - c)
            if norm(nominal_goal - pos) + agent_radius/2 < norm(nominal_goal - c)
                # Just keep going toward goal (on the clear side)--this statement should never be reached
                mode = :MOVE_TOWARD_GOAL
            elseif dmin < detour_horizon
                # detour
                if dmin < 2*proximity_tolerance
                    # right at circle---just turn right to skim
                    mode = :MOVE_ALONG_CIRCLE
                else
                    # Pick a tangent point to shoot for
                    mode = :MOVE_TANGENT
                end
            else
                mode = :MOVE_TOWARD_GOAL
            end
        else
            # INSIDE CIRCLE
            mode = :EXIT_CIRCLE
        end
    end
    policy.mode = mode
end

function tangent_bug_policy!(policy, circles, pos, nominal_goal, parent_step_active)
    @unpack planning_radius, detour_horizon, proximity_tolerance, buffer,
        agent_radius, vmax, dt = policy

    c = nothing
    r = nothing
    id = nothing
    dmin = Inf
    id, circ, waypoint = get_closest_interfering_circle(policy,circles,pos,nominal_goal)
    if !(circ === nothing)
        c = get_center(circ)
        r = get_radius(circ)
        dmin = norm(c - pos) - r
    end
    # select operating mode
    mode = set_policy_mode!(policy,circ,pos,nominal_goal, parent_step_active)

    # Select waypoint
    goal = nominal_goal
    if mode == :WAIT_OUTSIDE
        if norm(nominal_goal - c) > 1e-3
            goal = c + normalize(nominal_goal - c)*r
        else
            goal = c + normalize(pos - c)*r
        end
    elseif mode == :MOVE_TOWARD_GOAL
        goal = nominal_goal
    elseif mode == :MOVE_ALONG_CIRCLE
        dvec = normalize(pos - c) * (r + proximity_tolerance)
        # compute sector to traverse
        dr = vmax * dt
        dθ = 2*sin(0.5*dr / r)
        goal = c + [cos(dθ) -sin(dθ); sin(dθ) cos(dθ)] * dvec
    elseif mode == :MOVE_TANGENT
        vec = c - pos # vector from pos to circle center
        ψ = asin(r/norm(vec)) # yaw angle of right tangent line
        dθ = π/2 - ψ # CCW angular offset to tangent point
        dvec = normalize(pos - c) * r
        goal = c + [cos(dθ) -sin(dθ); sin(dθ) cos(dθ)] * dvec
        # if goal causes intersection with another circle, choose waypoint instead
        new_id, _, _ = get_closest_interfering_circle(policy,circles,pos,goal)
        if !(new_id === nothing || new_id == id)
            policy.mode = :MOVE_TOWARD_GOAL
            goal = waypoint
        end
    elseif mode == :EXIT_CIRCLE
        vec = pos - c
        if norm(vec) < 1e-3
            goal = nominal_goal
        else
            goal = c + (r + 2*proximity_tolerance) * normalize(vec)
        end
    else
        throw(ErrorException("Unknown controller mode $mode"))
    end
    # compute desired velocity
    vec = goal - pos
    if norm(vec) > vmax*dt
        vel = vmax * normalize(vec)
    else
        vel = vec / dt
    end
    policy.cmd = Twist(SVector(vel[1:2]...,0.0),SVector(0.0,0.0,0.0))
    # @show mode, dmin
    return goal
end

query_policy_for_goal!(policy::TangentBugPolicy,args...) = tangent_bug_policy!(policy,args...)
