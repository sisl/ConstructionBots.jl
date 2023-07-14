using Test

Revise.includet("/home/kylebrown/.julia/dev/ConstructionBots/src/highway.jl")
let 

    refline = Arc(SVector(0.0,0.0),1.0,0.0,2π)
    @test isapprox(norm(start_pt(refline) .- pt_from_arc_length(refline,0)),0.0,atol=1e-9)
    @test isapprox(norm(end_pt(refline) .- pt_from_arc_length(refline,arclength(refline))),0.0,atol=1e-9)
    @test isapprox(norm([0.9,0.0] .- pt_from_frenet(refline,Frenet(0,0.1))),0.0,atol=1e-9)
    @test isapprox(norm([-0.9,0.0] .- pt_from_frenet(refline,Frenet(π,0.1))),0.0,atol=1e-9)

    coords = proj_pt_to_refline(refline,start_pt(refline))
    @test isapprox(coords.s, 0, atol=1e-9)
    @test isapprox(coords.e, 0, atol=1e-9)

    refline = GeometryBasics.Line(Point(0.0,0.0),Point(1.0,0.0))
    @test isapprox(norm(start_pt(refline) .- pt_from_arc_length(refline,0)),0.0,atol=1e-9)
    @test isapprox(norm(end_pt(refline) .- pt_from_arc_length(refline,arclength(refline))),0.0,atol=1e-9)
    @test isapprox(norm(start_pt(refline) .+ [0.0,0.1] .- pt_from_frenet(refline,Frenet(0.0,0.1))),0.0,atol=1e-9)

    coords = proj_pt_to_refline(refline,start_pt(refline))
    @test isapprox(coords.s, 0, atol=1e-9)
    @test isapprox(coords.e, 0, atol=1e-9)

end

let 
    coords = Frenet(0.0,0.0)
    next_coords = move_along_action(coords,Frenet(1.0,1.0),1.0)
    @test isapprox(next_coords.s, 1.0)
    @test isapprox(next_coords.e, 1.0)

end

let 
    for p in -2π:0.1:2π
        arc = Arc(SVector(0.0,0.0),1.0,p,p+π)
        @test isapprox(norm(start_pt(arc) - pt_from_relative_angle(arc,0.0)),0.0,atol=1e-10)
        @test isapprox(norm(end_pt(arc) - pt_from_relative_angle(arc,π)),0.0,atol=1e-10)
        @test isapprox(norm(end_pt(arc) - pt_from_arc_length(arc,arclength(arc))),0.0,atol=1e-10)
        @test isapprox(norm([0.0,1.0] - pt_from_absolute_angle(arc,π/2)),0.0,atol=1e-10)
        @test isapprox(norm([0.0,1.0] - pt_from_relative_angle(arc,π/2-p)),0.0,atol=1e-10)
        @test isapprox(norm([0.0,1.0] - pt_from_arc_length(arc,arc.radius*(π/2-p))),0.0,atol=1e-10)
        @test isapprox(norm([0.0,1.0] - pt_from_arc_length(arc,arclength_from_absolute_angle(arc,π/2))),0.0,atol=1e-10)
    end
    
end

let 
    arc = Arc(SVector(0.0,0.0),1.0,0.0,π)
    @test isapprox(norm([0.0,1.0] - pt_from_absolute_angle(arc,π/2)),0.0,atol=1e-10)
    # @show  pt_from_absolute_angle(arc,π/2)

    coords = proj_pt_to_arc(arc,Point2(2.0,0.0))
    @test isapprox(coords.e,-1.0)
    @test isapprox(coords.s,0.0)

    coords = proj_pt_to_arc(arc,Point2(2.0,-1.0))
    @test isapprox(coords.s,0.0)

    coords = proj_pt_to_arc(arc,Point2(2.0,2.0))
    @test isapprox(coords.e,-(2*sqrt(2)-1))
    @test isapprox(coords.s,π/4)

    coords = proj_pt_to_arc(arc,Point2(-2.0,0.0))
    @test isapprox(coords.e,-1.0)
    @test isapprox(coords.s,π)

    coords = proj_pt_to_arc(arc,Point2(-2.0,-1.0))
    # @test isapprox(t,1.0)
    @test isapprox(coords.s,π)

end

# test Lane Reservations
let 
    agent_id1 = TemplatedID{Int}(1)
    agent_id2 = TemplatedID{Int}(2)
    agent_id3 = TemplatedID{Int}(3)
    res1 = LaneReservation(LaneID(1), agent_id1, 0.0, 1.0)
    res2 = LaneReservation(LaneID(1), agent_id2, 0.1, 1.0)
    res3 = LaneReservation(LaneID(1), agent_id3, 1.1, 1.2)
    @test !(res1 < res2)
    @test !(res2 < res1)
    @test res1 < res3

    lane = Lane()
    @test add_lane_reservation!(lane,res1)
    @test !add_lane_reservation!(lane,res2)
    @test add_lane_reservation!(lane,res3)
    @test has_lane_reservation(lane,agent_id1)
    @test find_reservation_index(lane,agent_id1) == 1
    @test find_reservation_index(lane,agent_id3) == 2
    @test has_lane_reservation(lane,agent_id3)
    remove_lane_reservation!(lane,agent_id1)
    @test !has_lane_reservation(lane,agent_id1)
    remove_lane_reservation!(lane,agent_id3)
    @test !has_lane_reservation(lane,agent_id3)

    @test add_lane_reservation!(lane,res1)
    clear_lane_reservations!(lane)
    @test !has_lane_reservation(lane,agent_id1)

    # push!(lane.reservations,res1)
    # idx = searchsortedfirst(lane.reservations,res2)
    # res2 < lane.reservations[idx]
    # idx = searchsortedfirst(lane.reservations,res3)
    # searchsorted(lane.reservations,res3)

end

let 
    highway = Highway()
    lane = Lane(
        refline = Arc(SVector(0.0,0.0),1.0,0.0,2π)
    )
    add_lane!(highway,lane)
    split_lane!(highway,lane,π)
    lane, new_lane = split_lane_at_absolute_angle!(highway,lane,π/2)

    outneighbors(highway,lane)
    for n in get_nodes(highway)
        # @show n,n.refline
        for np in node_iterator(highway,outneighbors(highway,n))
            @test isapprox(np.refline.a,n.refline.b)
        end
    end
    # @show end_pt(lane.refline)
    # @show start_pt(lane.refline)

    bridge = add_straight_lane_bridge!(highway,lane,new_lane)
    @test nv(highway) == 4
    @test indegree(highway,bridge) == 1
    @test outdegree(highway,bridge) == 1

end

let
    highway = Highway()
    lane1 = Lane(refline=Arc(SVector(0.0,0.0),2.0,0.0,π))
    lane2 = Lane(refline=Arc(SVector(0.0,0.5),1.0,0.0,π))
    add_lane!(highway,lane1)
    add_lane!(highway,lane2)
    H1 = deepcopy(highway)
    split_and_bridge_by_arc_length!(highway,lane1,lane2,π,π/2)
    @test nv(highway) == 5
    double_split_and_bridge_by_arc_length!(H1,lane1,lane2,[π,π+0.3],[π/2,π/2+0.2])
    @test nv(H1) == 8
    for hwy in [highway,H1]
        for n in get_nodes(hwy)
            @test indegree(hwy,n) + outdegree(hwy,n) > 0
        end
        @test is_connected(hwy)
    end

end
