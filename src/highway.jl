import LibSpatialIndex
using LinearAlgebra
using GeometryBasics
using StaticArrays
using LightGraphs, GraphUtils
using Parameters
using Test

"""
    Frenet

Frenet coordinates to define 2D position in curvilinear coordinate system.
Positive e direction is to the left of the curve.
"""
struct Frenet
    s::Float64
    e::Float64
end

struct LaneID <: AbstractID
    id::Int
end

struct Arc
    center::Point2{Float64}
    radius::Float64
    a::Float64
    b::Float64
end
arclength(a::Arc)   = a.radius*(a.b-a.a) 
start_pt(a::Arc)    = Point2((a.center + a.radius * [cos(a.a),sin(a.a)])...)
end_pt(a::Arc)      = Point2((a.center + a.radius * [cos(a.b),sin(a.b)])...)
pt_from_absolute_angle(a::Arc,θ,r=a.radius) = a.center + r * Point2(cos(θ),sin(θ))
pt_from_relative_angle(a::Arc,θ,r=a.radius) = pt_from_absolute_angle(a,a.a+θ,r)
pt_from_arc_length(a::Arc,s,r=a.radius) = pt_from_absolute_angle(a,a.a+(a.b-a.a)*s/arclength(a),r)
# Frenet positive axis is to left
pt_from_frenet(a::Arc,coords::Frenet) = pt_from_arc_length(a,coords.s,a.radius-coords.e)

function arclength_from_absolute_angle(arc::Arc,θ)
    return arc.radius*wrap_to_2pi(angular_difference(arc.a,θ))
end

arclength(a::GeometryBasics.Line)   = norm(a.points[2]-a.points[1])
start_pt(a::GeometryBasics.Line)         = a.points[1]
end_pt(a::GeometryBasics.Line)           = a.points[2]
direction_vec(a::GeometryBasics.Line) = normalize(end_pt(a) - start_pt(a))
pt_from_arc_length(a::GeometryBasics.Line,s) = start_pt(a) + (end_pt(a) - start_pt(a))*s/arclength(a)
function pt_from_frenet(a::GeometryBasics.Line,coords::Frenet) 
    pt_from_arc_length(a,coords.s) + coords.e * [0.0 -1.0; 1.0 0.0] * direction_vec(a)
end

"""
    proj_pt_to_arc(arc::Arc,pt)

Return Frenet coordinate of pt in arc frame.
"""
function proj_pt_to_arc(arc::Arc,pt,clip=true)
    v = pt - arc.center
    t = arc.radius - norm(v) # assumes CCW, positive to the left
    θ = wrap_to_2pi(atan(v[2],v[1]))
    da = angular_difference(arc.a,θ)
    db = angular_difference(θ,arc.b)
    d = arc.b - arc.a # in case b == 2π
    if clip && wrap_to_2pi(da) + wrap_to_2pi(db) > wrap_to_2pi(d)
        if abs(da) < abs(db)
            s = 0.0
        else
            s = arclength(arc)
        end
    else
        s = arclength(arc) * wrap_to_2pi(da)/d
    end
    return Frenet(s, t)
end

function proj_pt_to_line(line::GeometryBasics.Line,pt,clip=true)
    v = pt - start_pt(line)
    s = dot(v,direction_vec(line))
    e = norm(v - start_pt(line) + s*direction_vec(line))
    Frenet(s,e)
end
proj_pt_to_refline(a::Arc,args...) = proj_pt_to_arc(a,args...)
proj_pt_to_refline(a::GeometryBasics.Line,args...) = proj_pt_to_line(a,args...)

"""
    move_along(coords::Frenet,s_dot,e_dot,dt)

Return next Frenet coordinates from moving at constant velocity in Frenet frame 
for `dt` seconds.
"""
function move_along(coords::Frenet,s_dot,e_dot,dt)
    Frenet(coords.s+s_dot*dt,coords.e+e_dot*dt)
end

"""
    move_along(coords::Frenet,vel::Frenet,dt)
"""
move_along(coords::Frenet,vel::Frenet,dt) = move_along_action(coords,vel.s,vel.e,dt)

"""
    LaneReservation

Represents a reservation of part of a lane.
"""
struct LaneReservation
    lane_id::LaneID
    agent_id::AbstractID
    s_lo::Float64
    s_hi::Float64 # must be greater than or equal to s_lo
end
Base.isless(a::LaneReservation,b::LaneReservation) = a.s_hi < b.s_lo
Base.isgreater(a::LaneReservation,b::LaneReservation) = a.s_lo > b.s_hi

"""
    LaneState

Encodes lane id and the frenet coordinates along that lane
"""
struct LaneState
    lane_id::LaneID # if lane_id is iinvalid, agent is not currently on highway
    coords::Frenet
end

"""
    HighwayAgentState

Encodes an agent's lane state, goals, speed limit, etc.
"""
mutable struct HighwayAgentState
    # state variables
    lane_states ::Dict{LaneID,LaneState} # May have multiple simultaneous lane associations, but only one target lane
    on_highway  ::Bool
    plan        ::Vector{LaneID} # where is agent trying to go overall?
    target      ::LaneID # where is agent trying to go currently? 
    # fixed params
    agent_id    ::AbstractID
    max_speed   ::Float64
    radius      ::Float64
end
is_on_highway(s::HighwayAgentState) = s.on_highway
function generate_reservation(s::HighwayAgentState,lane_state::LaneState)
    LaneReservation(
        lane_state.lane_id,
        s.agent_id,
        lane_state.coords.s - s.radius,
        lane_state.coords.s + s.radius,
        )
end
function generate_reservations(s::HighwayAgentState)
    Dict(lane_id=>generate_reservation(s,ls) for (lane_id,ls) in s.lane_states)
end
# struct ParkingZone
#     circle::GeometryBasics.Circle
#     occupied::Bool
# end

@with_kw mutable struct Lane
    id::LaneID = get_unique_id(LaneID)
    refline::Union{GeometryBasics.Line,Arc} = GeometryBasics.Line(Point(0.0,0.0),Point(1.0,1.0))
    width::Float64                          = 1.0
    max_speed::Float64                      = 1.0
    reservations::Vector{LaneReservation}   = Vector{LaneReservation}()  # sorted vector of lane reservations
    agent_ids::Set{AbstractID}              = Set{AbstractID}()
end
GraphUtils.node_id(lane::Lane) = lane.id
function check_lane_available(lane::Lane,res::LaneReservation)
    idx = searchsortedfirst(lane.reservations,res)
    return (idx > length(lane.reservations)) || (res < lane.reservations[idx])
end
function add_lane_reservation!(lane::Lane,res::LaneReservation)
    if res.agent_id in lane.agent_ids
        @info "Agent $(agent_id) already has a reservation"
    elseif isempty(lane.reservations) || check_lane_available(lane,res)
        insert_to_sorted_array!(lane.reservations,res)
        push!(lane.agent_ids,res.agent_id)
        return true
    end
    return false
end
find_reservation_index(lane::Lane,agent_id) = findfirst(res->res.agent_id == agent_id, lane.reservations)
function find_lane_reservation(lane::Lane,agent_id)
    idx = find_reservation_index(lane,agent_id)
    if isnothing(idx)
        return lane.reservations[idx]
    end
    return LaneReservation()
end
has_lane_reservation(lane::Lane,agent_id) = agent_id in lane.agent_ids
function remove_lane_reservation!(lane::Lane,agent_id)
    idx = findfirst(res->res.agent_id == agent_id,lane.reservations)
    if !isnothing(idx)
        deleteat!(lane.reservations, idx)
    end
    delete!(lane.agent_ids,agent_id)
end
function clear_lane_reservations!(lane::Lane)
    empty!(lane.agent_ids)
    empty!(lane.reservations)
end

"""
    LaneJunction

Encodes a junction where `parent` merges into `child`. The junction is at arc
longitudinal coordinate `s_parent` on parent and coordinate `s_child` on child.
"""
struct LaneJunction
    parent::Lane
    child::Lane
    s_parent::Float64
    s_child::Float64
end

"""
    update_lane_reservation!(lane::Lane,res::LaneReservation)

Try to replace an agent's previous reservation with a new reservation.
"""
function update_lane_reservation!(lane::Lane,res::LaneReservation)
    if has_lane_reservation(lane,res.agent_id)
        old_res = find_lane_reservation(lane,res.agent_id)
        remove_lane_reservation!(lane,res.agent_id)
        if add_lane_reservation!(lane,res)
            return true
        else
            add_lane_reservation!(lane,old_res) # put old_res back in
            return false
        end
    else
    end
end

"""
    step_highway!(env,highway,states::Dict{AbstractID,Pair{LaneID,LaneState}})

Q: How to update all reservations across the entire highway? 
A: Just don't allow overlap with the previous time step, but try to update 
reservations in order of slowest-moving agents first.
"""
function step_highway!(env,highway,states::Dict{AbstractID,HighwayAgentState})
    # TODO sort by agent speed
    for (agent_id,state) in states
        step_agent_along_highway!(env,highway,agent_id,state)
    end
end

function step_agent_along_highway!(env,highway,agent_id,state::HighwayAgentState)
    if is_on_highway(state)
        return state # no change
    end
    # HIGH_LEVEL_BEHAVIORS: MOVE_ALONG, EXIT (to new lane), OFF_ROAD, PARK ?
    # if MOVE_ALONG  
    # elseif EXIT -> where?
    # 
end

"""
    NestedHighway

A nested highway structure.

Q: How to correctly associate lanes to staging areas?
A: zone_dict: Dict mapping AssemblyID => LaneID of the circular lane surrounding 
that assembly's staging area. Probably also want a dict mapping LaneID => AssemblyID

Q: How to conveniently represent the nested topology of the highway, including the 
arc length coordinates at which lanes merge into each other?
A: As follows:
    child_entrances: 
        child_circle => [s_self=>child_entrance_lane, s_self=>other_entrance_lane]
        ...
    child_exits: 
        child_circle => [s_self=>child_exit_lane, s_self=>other_exit_lane]
        ...
    parent_entrances: 
        parent_circle => [s_self=>parent_entrance_lane, s_self=>parent_entrance_lane]
        ...
    parent_exits: 
        parent_circle => [s_self=>parent_exit_lane, s_self=>other_exit_lane]
        ...

Q: Given a robot's current position and goal position within a staging area, how 
to plan a path through the highway?
A: As follows:
    # 1. identify circuit lane associated with goal staging area
    goal_lane = zone_dict[assembly_id] 
    θ_exit = atan(goal_pt .- staging_circle.center)
    goal_s = arclength_from_absolute_angle(goal_lane.refline, θ_exit)
    # 2. extract sequence of nested goals by walking up scene_tree
    goal_sequence = [zone_dict[parent_id] in for parent_id in recurse_parents(scene_tree,assembly_id)]
    # 3. extract lane sequence from goal set
    lane_sequence = []
    for 
"""
mutable struct NestedHighway <: AbstractTreeNode{ID}
    # tree node fields
    id              ::ID
    parent          ::NestedHighway
    children        ::Dict{ID,NestedHighway}
    #
    lane            ::Lane # Lane around exterior
    child_entrances ::Dict{AbstractID,Vector{LaneJunction}} # straight line entrances from parent, stored by s value
    child_exits     ::Dict{AbstractID,Vector{LaneJunction}} # straight line exits from parent, stored by s value
    parent_entrances::Dict{AbstractID,Vector{LaneJunction}} # straight line entrances from parent, stored by s value
    parent_exits    ::Dict{AbstractID,Vector{LaneJunction}} # straight line exits from parent, stored by s value
    # inner_parking   ::Vector{Pair{Float64,}}
    # outer_parking   ::Vector{Pair{Float64,}}
    # These belong to the global highway
    zone_map        ::Dict{AbstractID,LaneID} # maps from zone id to lane id
    lane_map        ::Dict{LaneID,AbstractID} # maps from lane id to zone id
end
get_parent_entrance_lanes(hwy::CircleHighway)   = hwy.entrances
get_parent_exit_lanes(hwy::CircleHighway)       = hwy.exits





@with_kw struct Highway <: AbstractCustomNGraph{DiGraph,Lane,LaneID}
    # Graph Stuff
    graph       ::DiGraph                   = DiGraph()
    nodes       ::Vector{Lane}              = Vector{Lane}()
    vtx_map     ::Dict{LaneID,Int}          = Dict{LaneID,Int}()
    vtx_ids     ::Vector{LaneID}            = Vector{LaneID}() # maps vertex uid to actual graph node
    # Spatial Index stuff
    rtree       ::LibSpatialIndex.RTree     = LibSpatialIndex.RTree(2)
    # maps from staging zone to Set{LaneID}
    zone_map    ::Dict{AbstractID,LaneID}   = Dict{AbstractID,LaneID}() # maps from zone id to lane id
    lane_map    ::Dict{LaneID,AbstractID}   = Dict{LaneID,AbstractID}() # maps from lane id to zone id

end
GraphUtils.add_node!(h::Highway,l::Lane) = add_node!(h,l,node_id(l))
function add_lane!(h::Highway,l::Lane,id=node_id(l))
    add_node!(h,l,id)
    # add to spatial index
end

split_lane!(highway,lane,s) = split_lane!(lane.refline,highway,lane,s)

function split_lane!(arc::Arc,highway,lane,s)
    c = (s / arclength(arc)) * (arc.b - arc.a)
    new_lane = Lane(
        lane,
        id=get_unique_id(LaneID),
        refline=Arc(arc.center,arc.radius,c,arc.b),
        )
    lane.refline = Arc(arc.center,arc.radius,arc.a,c)
    # add_node!(highway,new_lane,node_id(new_lane))
    add_lane!(highway,new_lane)
    for n in node_iterator(highway,outneighbors(highway,node_id(lane)))
        rem_edge!(highway,lane,n)
        add_edge!(highway,new_lane,n)
    end
    add_edge!(highway,lane,new_lane)
    lane, new_lane
end

split_lane_by_arc_length!(highway,lane,s) = split_lane!(highway,lane,s)

function split_lane_at_absolute_angle!(highway,lane,θ)
    arc = lane.refline
    @assert isa(arc,Arc)
    s = arclength(arc) * ((θ - arc.a) / (arc.b-arc.a))
    split_lane!(highway,lane,s)
end

function add_straight_lane_bridge!(highway,lane1,lane2;
        width=lane1.width,
    )
    p1 = end_pt(lane1.refline)
    p2 = start_pt(lane2.refline)
    bridge = Lane(
        refline=GeometryBasics.Line(p1,p2),
        width=width,
    )
    # add_node!(highway,bridge)
    add_lane!(highway,bridge)
    add_edge!(highway,lane1,bridge)
    add_edge!(highway,bridge,lane2)
    return bridge
end

center_diff(a::Arc,b::Arc) = b.center - a.center

"""
    add_nested_junction!(highway,
        outer_lane,
        inner_lane,
        )

Adds a in+out junction between two arc lanes, `outer_lane` and `inner_lane`
"""
function add_nested_junction!(highway,
        lane1,
        lane2,
        ;
        s1=arclength_from_absolute_angle(lane1.refline,atan(
            reverse(center_diff(lane1.refline,lane2.refline))...)),
        s2=arclength_from_absolute_angle(lane2.refline,atan(
            reverse(center_diff(lane1.refline,lane2.refline))...)),
        lane_width = lane2.width
        )
    @assert isa(lane2.refline,Arc) && isa(lane1.refline,Arc)
    p1 = pt_from_arc_length(lane1.refline,s1)
    p2 = pt_from_arc_length(lane2.refline,s2)
    p1, p2
    # s1,s2
    # @assert lane2.refline.a <= θ_center <= lane2.refline.b
    # @assert lane1.refline.a <= θ_center <= lane1.refline.b
    # r = lane2.refline.radius
    # b = lane_width
    # dθ = 2*asin(b/(2*r)) # angular offset between the incoming and outgoing lanes
    # outer1, outer2 = split_lane!(highway,lane2,θ-dθ/2)
    # outer2, outer3 = split_lane!(highway,outer2,θ+dθ/2)
    # inner1, inner2 = split_lane!(highway,lane2,θ-dθ/2)
    # inner2, inner3 = split_lane!(highway,outer2,θ+dθ/2)

end

"""
    split_and_bridge!(highway,lane1,lane2,s1,s2)

Split lane1 at arc length parameter s1, and lane2 at arc length parameter s2
"""
function split_and_bridge_by_arc_length!(highway,lane1,lane2,s1,s2)
    lane1_a, lane1_b = split_lane_by_arc_length!(highway,lane1,s1)
    lane2_a, lane2_b = split_lane_by_arc_length!(highway,lane2,s2)
    add_straight_lane_bridge!(highway,lane1_a,lane2_b)
end

function double_split_and_bridge_by_arc_length!(highway,lane1,lane2,s1,s2)
    lane1_b, lane1_c = split_lane_by_arc_length!(highway,lane1,s1[2])
    lane1_a, lane1_b = split_lane_by_arc_length!(highway,lane1_b,s1[1])

    lane2_b, lane2_c = split_lane_by_arc_length!(highway,lane2,s2[2])
    lane2_a, lane2_b = split_lane_by_arc_length!(highway,lane2_b,s2[1])

    add_straight_lane_bridge!(highway,lane1_a,lane2_b)
    add_straight_lane_bridge!(highway,lane2_a,lane1_c)
end

"""
    construct_nested_highway

Build a nested highway with CCW lanes
"""
function construct_nested_highway(
        sched,
        scene_tree,
        staging_zones, # Final staging areas (without transformation)
        ;
        lane_width=1.0,
        max_speed=1.0,
        gkey = HypersphereKey(),
    )
    # ensure that scene_tree represents fully connected assembly
    HG.jump_to_final_configuration!(scene_tree;set_edges=true)
    # preprocessing
    dropoff_zones = Dict{AbstractID,Point2{Float64}}()
    inner_staging_zones = Dict{AbstractID,Ball2}() # inner zones in global frame
    outer_staging_zones = Dict{AbstractID,Ball2}() # outer zones in global frame
    for n in get_nodes(sched)
        if matches_template(DepositCargo,n)
            pt = HG.project_to_2d(global_transform(goal_config(n)).translation)
            # may need to account for TransportUnitNode center...
            dropoff_zones[cargo_id(entity(n))] = Point2{Float64}(pt...)
        elseif matches_template(AssemblyComplete,n)
            # get bounding circle from final build step
            inner_zone = get_cached_geom(node_val(n).inner_staging_circle)
            outer_zone = get_cached_geom(node_val(n).outer_staging_circle)
            inner_staging_zones[node_id(entity(n))] = inner_zone
            outer_staging_zones[node_id(entity(n))] = outer_zone
        end
    end
    dropoff_zones, inner_staging_zones, outer_staging_zones
    # initialize highway
    highway = Highway()
    # starting with leaf nodes:
    for n in node_iterator(sched, topological_sort_by_dfs(sched))
        if matches_template(AssemblyComplete,n)
            assembly = entity(n)
            # create outer lane surrounding circle
            outer_zone = outer_staging_zones[node_id(assembly)]
            inner_zone = inner_staging_zones[node_id(assembly)]
            # ctr = goal_config(n)        # center of lane arc
            r = get_radius(outer_zone)  +  # radius of lane arc
            # if has child assemblies
            if !isempty(assembly_components(assembly))
                # add lane for own inner zone from final build step
                # add junctions to/from child assembly staging areas
            end
        end
    end
    # create full circular lane around 
end

# """
#     construct_highway(zones::Dict;
# """
# function construct_highway(zones::Dict;
#         lane_width=1.0,
#         max_speed=1.0,
#         )
#     highway = Highway()
#     lane_dict = Dict(id=>Dict(:incoming=>Set{LaneID}(),:outgoing=>Set{LaneID}()) for id in keys(zones))
#     # Add straight lanes
#     # for all circles, find CCW tangent lines
#     for (i,circ1) in zones
#         for (j,circ2) in zones
#             if i != j
#                 continue
#             end
#             line = tangent_line_on_circs(circ1,circ2)
#             clear_path = true
#             for (k,circ) in zones
#                 if (k == i) || (k == j)
#                     continue
#                 end
#                 if circle_intersection_with_line(inflate(circ,lane_width),line)
#                     clear_path = false
#                     break
#                 end
#             end
#             if clear_path
#                 lane = Lane(
#                     refline=line,
#                     width=lane_width,
#                     max_speed=max_speed,
#                     zones=Set([i,j])
#                 )
#                 add_node!(highway,lane,node_id(lane))
#                 push!(lane_dict[:outgoing][i],node_id(lane))
#                 push!(lane_dict[:incoming][j],node_id(lane))
#             end
#         end
#     end
#     # TODO curved lanes
#     for (id,circ) in zones
#         angles = Vector{Pair{LaneID,Float64}}()
#         for lane_id in lane_dict[id][:incoming] 
#             lane = get_node(highway,lane_id)
#             push!(angles,lane_id=>atan(reverse(end_pt(lane))...))
#             # iteratively split lanes
#         end
#     end
#     return highway
# end

inflate(c::GeometryBasics.Circle,r::Float64) = GeometryBasics.Circle(c.center,c.r+r)

function get_tangent_to_two_circles(c1,r1,c2,r2)
    if r1 > r2
        p2, p1 = get_tangent_to_two_circles(c2,r2,c1,r1)
        p1 = c1 .+ (c1 .- p1)
        p2 = c2 .+ (c2 .- p2)
        return p1, p2
    end
    v = c2-c1  # vector from c1 to c2
    dr = r2-r1 # difference in radius
    θ = asin(dr/norm(v)) + π/2
    p = RotMatrix(Angle2d(θ)) * normalize(v)
    p1 = c1 .+ p * r1
    p2 = c2 .+ p * r2
    return p1, p2
end

"""
    tangent_line_on_circs(circ1,circ2)

Return line tangent to both circles, pointing in CCW direction from circ1 to 
circ2.
"""
function tangent_line_on_circs(circ1,circ2)
    p1, p2 = get_tangent_to_two_circles(
        get_center(circ1),
        get_radius(circ1),
        get_center(circ2),
        get_radius(circ2),
        )
    GeometryBasics.Line(Point(p1...),Point(p2...))
end
