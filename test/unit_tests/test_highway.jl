let 
    highway = Highway()
    lane = Lane(
        refline = Arc(SVector(0.0,0.0),1.0,0.0,2π)
    )
    add_node!(highway,lane,node_id(lane))
    split_lane!(highway,lane,π)
    lane, new_lane = split_lane_at_absolute_angle!(highway,lane,π/2)

    outneighbors(highway,lane)
    for n in get_nodes(highway)
        @show n,n.refline
        for np in node_iterator(highway,outneighbors(highway,n))
            @test isapprox(np.refline.a,n.refline.b)
        end
    end
    @show end_pt(lane.refline)
    @show start_pt(lane.refline)

    bridge = add_straight_lane_bridge!(highway,lane,new_lane)
    @test nv(highway) == 4
    @test indegree(highway,bridge) == 1
    @test outdegree(highway,bridge) == 1

end