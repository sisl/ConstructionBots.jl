struct LaneID <: AbstractID
    id::Int
end

struct Arc
    center::SVector{2,Float64}
    radius::Float64
    a::Float64
    b::Float64
end
arclength(a::Arc)   = a.radius*(b-a) 
start_pt(a::Arc)    = a.center + a.radius * [cos(a.a),sin(a.a)]
end_pt(a::Arc)      = a.center + a.radius * [cos(a.b),sin(a.b)]

arclength(a::GeometryBasics.Line)   = norm(a.points[2]-a.points[1])
start_pt(a::GeometryBasics)         = a.points[1]
end_pt(a::GeometryBasics)           = a.points[2]

@with_kw mutable struct Lane
    id::LaneID = get_unique_id(LaneID)
    refline::Union{GeometryBasics.Line,Arc} = GeometryBasics.Line(Point(0.0,0.0),Point(1.0,1.0))
    width::Float64 = 1.0
    max_speed::Float64 = 1.0
    length::Float64 = arclength(refline)
    # something about neighbors / adjacent zones
    zones::Set{AbstractID} = Set{AbstractID}()
end
GraphUtils.node_id(lane::Lane) = lane.id

@with_kw struct Highway <: AbstractCustomNGraph{DiGraph,Lane,LaneID}
    graph               ::DiGraph               = DiGraph()
    nodes               ::Vector{Lane}          = Vector{Lane}()
    vtx_map             ::Dict{LaneID,Int}      = Dict{LaneID,Int}()
    vtx_ids             ::Vector{LaneID}        = Vector{LaneID}() # maps vertex uid to actual graph node
end

function construct_highway(zones::Dict;
        lane_width=1.0,
        max_speed=1.0,
        )
    highway = Highway()
    lane_dict = Dict(id=>Dict(:incoming=>Set{LaneID}(),:outgoing=>Set{LaneID}()) for id in keys(zones))
    # Add straight lanes
    # for all circles, find CCW tangent lines
    for (i,circ1) in zones
        for (j,circ2) in zones
            if i != j
                continue
            end
            line = tangent_line_on_circs(circ1,circ2)
            clear_path = true
            for (k,circ) in zones
                if (k == i) || (k == j)
                    continue
                end
                if circle_intersection_with_line(inflate(circ,lane_width),line)
                    clear_path = false
                    break
                end
            end
            if clear_path
                lane = Lane(
                    refline=line,
                    width=lane_width,
                    max_speed=max_speed,
                    zones=Set([i,j])
                )
                add_node!(highway,lane,node_id(lane))
                push!(lane_dict[:outgoing][i],node_id(lane))
                push!(lane_dict[:incoming][j],node_id(lane))
            end
        end
    end
    # TODO curved lanes
    for (id,circ) in zones
        angles = Vector{Pair{LaneID,Float64}}()
        for lane_id in lane_dict[id][:incoming] 
            lane = get_node(highway,lane_id)
            push!(angles,lane_id=>atan(reverse(end_pt(lane))...))
            # iteratively split lanes
        end
    end
    return highway
end

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
