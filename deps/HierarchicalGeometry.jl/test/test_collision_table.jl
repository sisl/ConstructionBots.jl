using ConstructionBots
using HierarchicalGeometry
using DataStructures
using LightGraphs
using GraphUtils

# collision_table = CustomNGraph{Graphs.Graph,PriorityQueue{AbstractID,Float64},AbstractID}()

function min_time_to_collision(x1,x2,vmax1=1.0,vmax2=1.0)
    d_min = distance_lower_bound(x1,x2)
    v_max = vmax1 + vmax2
    dt = d_min / v_max
end
function min_time_to_collision(agent1,agent2,gkey=HypersphereKey())
    geom1 = get_cached_geom(agent1,gkey)
    vmax1 = ConstructionBots.get_rvo_max_speed(agent1)
    geom2 = get_cached_geom(agent2,gkey)
    vmax2 = ConstructionBots.get_rvo_max_speed(agent2)
    min_time_to_collision(geom1,geom2,vmax1,vmax2)
end

function init_collision_table(agents,t0=0.0;
        gkey=HypersphereKey()
    )
    table = CustomNGraph{Graphs.Graph,PriorityQueue{AbstractID,Float64},AbstractID}()
    for agent1 in agents
        pq = GraphUtils._node_type(table)()
        add_node!(table,pq,node_id(agent1))
        for agent2 in agents
            if !(agent1 === agent2)
                dt = min_time_to_collision(agent1,agent2)
                pq[node_id(agent2)] = t0 + dt
            end
        end
    end
    table
end

"""
    check_neighbors!(table,agent_id,t)

Check position of all neighbors that might be within `dt` seconds of collision
at time t
"""
function check_neighbors!(table,scene_tree,agent1,t,dt;
        gkey=HypersphereKey(),
    )
    pq = get_node(table,node_id(agent1))
    neighbors = []
    while !isempty(pq)
        id,tmin = dequeue_pair!(pq)
        if tmin > t + dt
            pq[id] = tmin
            break
        else
            agent2 = get_node(scene_tree,id)
            dt = min_time_to_collision(agent1,agent2)
            push!(neighbors,id=>t+dt)
        end
    end
    # add back into queue
    for (id,tmin) in neighbors
        pq[id] = tmin
    end
    return neighbors
end


function update_collision_table!(table,scene_tree,agent_ids,t,dt)
    for agent in enumerate(agents)
        neighbors = check_neighbors!(table,scene_tree,agent,t,dt)
        for (id,tmin) in neighbors
            pq = get_node(table,id)
            pq[node_id(agent)] = tmin
        end
    end
    table
end

let
    scene_tree = SceneTree()
    for i in 1:10
        agent = RobotNode(RobotID(i),GeomNode(Ball2([0.0,0.0,0.0],1.0)))
        # add_node!(agent.geom)
        add_node!(scene_tree,agent)
        set_desired_global_transform!(agent,CT.Translation(i,0.0,0.0))
    end
    table = init_collision_table(get_nodes(scene_tree))
end
