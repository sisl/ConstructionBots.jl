struct LaneID <: AbstractID
    id::Int
end

struct Arc
    center::SVector{2,Float64}
    radius::Float64
    a::Float64
    b::Float64
end
arclength(a::Arc)   = a.radius*(a.b-a.a) 
start_pt(a::Arc)    = Point2((a.center + a.radius * [cos(a.a),sin(a.a)])...)
end_pt(a::Arc)      = Point2((a.center + a.radius * [cos(a.b),sin(a.b)])...)

function arclength_from_absolute_angle(arc::Arc,θ)
    if !(arc.a < θ < arc.b)
        @warn "!(arc.a ($(arc.a)) < θ ($(θ)) < arc.b ($(arc.b)))"
    end
    return arc.radius*(θ-arc.a)
end


arclength(a::GeometryBasics.Line)   = norm(a.points[2]-a.points[1])
start_pt(a::GeometryBasics.Line)         = a.points[1]
end_pt(a::GeometryBasics.Line)           = a.points[2]

@with_kw mutable struct Lane
    id::LaneID = get_unique_id(LaneID)
    refline::Union{GeometryBasics.Line,Arc} = GeometryBasics.Line(Point(0.0,0.0),Point(1.0,1.0))
    width::Float64 = 1.0
    max_speed::Float64 = 1.0
    length::Float64 = arclength(refline)
    # something about neighbors / adjacent zones
    # zone::AssemblyID = AssemblyID(-1) # point to zone?
end
GraphUtils.node_id(lane::Lane) = lane.id

@with_kw struct Highway <: AbstractCustomNGraph{DiGraph,Lane,LaneID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{Lane}          = Vector{Lane}()
    vtx_map             ::Dict{LaneID,Int}      = Dict{LaneID,Int}()
    vtx_ids             ::Vector{LaneID}        = Vector{LaneID}() # maps vertex uid to actual graph node
end
GraphUtils.add_node!(h::Highway,l::Lane) = add_node!(h,l,node_id(l))

split_lane!(highway,lane,s) = split_lane!(lane.refline,highway,lane,s)

function split_lane!(arc::Arc,highway,lane,s)
    c = (s / arclength(arc)) * (arc.b - arc.a)
    new_lane = Lane(
        lane,
        id=get_unique_id(LaneID),
        refline=Arc(arc.center,arc.radius,c,arc.b),
        )
    lane.refline = Arc(arc.center,arc.radius,arc.a,c)
    add_node!(highway,new_lane,node_id(new_lane))
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
        width=1.0,
    )
    p1 = end_pt(lane1.refline)
    p2 = start_pt(lane2.refline)
    bridge = Lane(
        refline=GeometryBasics.Line(p1,p2),
        width=width,
    )
    add_node!(highway,bridge)
    add_edge!(highway,lane1,bridge)
    add_edge!(highway,bridge,lane2)
    return bridge
end

"""
    add_nested_junction!(highway,
        outer_lane,
        inner_lane,
        )

Adds a in+out junction between two arc lanes, `outer_lane` and `inner_lane`
"""
function add_nested_junction!(highway,
        outer_lane,
        inner_lane,
        ;
        θ_center = atan(reverse(outer_lane.refline.center .- inner_lane.refline.center)...),
        lane_width = outer_lane.width
        )
    @assert isa(outer_lane.refline,Arc) && isa(inner_lane.refline,Arc)
    @assert outer_lane.refline.a <= θ_center <= outer_lane.refline.b
    @assert inner_lane.refline.a <= θ_center <= inner_lane.refline.b
    r = outer_lane.refline.radius
    b = lane_width
    dθ = 2*asin(b/(2*r)) # angular offset between the incoming and outgoing lanes
    outer1, outer2 = split_lane!(highway,outer_lane,θ-dθ/2)
    outer2, outer3 = split_lane!(highway,outer2,θ+dθ/2)
    inner1, inner2 = split_lane!(highway,outer_lane,θ-dθ/2)
    inner2, inner3 = split_lane!(highway,outer2,θ+dθ/2)

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
