using CRCBS, GraphUtils, CairoMakie, Makie, FactoryRendering

vtx_grid = initialize_dense_vtx_grid(4,4)
#  1   2   3   4
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
starts = [1,2,3,4,5,6,7,8]
goals = [13,14,15,16,9,10,11,12]
# graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
graph = construct_factory_env_from_vtx_grid(vtx_grid)
mapf = init_mapf_problem(graph,starts,goals)

solver = PIBTPlanner{Float64}()
set_iteration_limit!(solver,10)
set_verbosity!(solver,4)
solution, valid = pibt!(solver, mapf)
@show valid, convert_to_vertex_lists(solution)

vtx_lists = convert_to_vertex_lists(solution)
grid_vtxs = map(v->[v[1],v[2]], mapf.env.graph.vtxs)
paths = map(p->map(v->grid_vtxs[v],p),vtx_lists)
# scene = Scene()
scene = Scene(show_axis=false,scale_plot=false,resolution=(200,200))
t0 = 0.0
tf = length(paths[1]) + 1.0
pad = 0.75
corners = FactoryRendering.bbox_corners(grid_vtxs,pad)
poly!(scene,Point2f0.(corners), color=:gray)
scene = FactoryRendering.plot_grid_world!(scene,grid_vtxs)

t = Node(t0) # This is the life signal
positions = lift(t->FactoryRendering.env_state_snapshot(paths,t),t)
x = lift(positions->map(p->p[1],positions),positions)
y = lift(positions->map(p->p[2],positions),positions)
text_positions = map(i->lift(positions->tuple(positions[i]...),positions),1:length(paths))
scatter!(scene,x,y,marker=:circle,markersize=0.8,color=:red)
for i in 1:length(paths)
    text!(scene,string(i),textsize=0.4,align=(:center,:center), position=text_positions[i])
end
for (i,g) in enumerate(goals)
    text!(scene,string(i),textsize=0.2,align=(:left,:top),position=tuple((grid_vtxs[g] .+ [-0.45,0.48])...),overdraw=true)
end
scene

record(scene, "robots_moving.webm", 0:0.1:tf; framerate = 20) do dt
    t[] = t0 + dt
end
