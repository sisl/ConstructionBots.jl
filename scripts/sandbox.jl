using ConstructionBots
using LDrawParser
using HierarchicalGeometry

using LightGraphs, GraphUtils
using GeometryBasics, CoordinateTransformations, Rotations

using MeshCat
using Plots

# using Logging
# global_logger(SimpleLogger(stderr, Logging.Debug))

# filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","Millennium Falcon.mpd")
filename = joinpath(dirname(pathof(LDrawParser)),"..","assets","ATTEWalker.mpd")
model = parse_ldraw_file(filename)
populate_part_geometry!(model)

SCALE = 0.01
LDrawParser.change_coordinate_system!(model,ldraw_base_transform(),SCALE)
model

spec = ConstructionBots.construct_model_spec(model)
model_spec = ConstructionBots.extract_single_model(spec,"20009 - AT-TE Walker.mpd")

model_spec
outneighbors(model_spec,1)

collect(edges(bfs_tree(model_spec,1;dir=:in)))


model_graph = construct_assembly_graph(model)

model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
# @assert maximum(map(v->indegree(model_tree,v),vertices(model_tree))) == 1
print(model_tree,v->summary(v.val),"\t")

# NOTE about MeshCat: "The final pose of each geometry in the tree is just the
# composition of all of the transforms in the path from the root of the tree to
# that geometry."

# part_keys = sort(collect(keys(model.parts))[1:10])
# parts = Dict(k=>model.parts[k] for k in part_keys)
# transport_model = (
#     robot_radius = 15.0,
#     max_area_per_robot = 10000.0, #3000.0,
#     max_volume_per_robot = 1000000.0 #20000.0,
# )
# for (k,part) in parts
#     points = map(T, LDrawParser.extract_points(part))
#     if isempty(points)
#         println("Part $k has no geometry!")
#         continue
#     end
#     geom=map(SVector,points)
#     try
#         support_pts = HierarchicalGeometry.select_support_locations(
#             geom,transport_model)
#         polygon = VPolygon(convex_hull(map(p->p[1:2],geom)))
#
#         plt = plot(polygon,aspectratio=1,alpha=0.4)
#         scatter!(plt,map(p->p[1],geom),map(p->p[2],geom),label=k) #,legend=false)
#         plot!(plt,map(p->Ball2(p,transport_model.robot_radius),support_pts), aspectratio=1, alpha=0.4)
#         display(plt)
#     catch e
#         bt = catch_backtrace()
#         showerror(stdout,e,bt)
#     end
# end

vis = Visualizer()
render(vis)
vis_root = vis["root"]

T_base = CoordinateTransformations.Translation(0.0,0.0,0.0) ∘ CoordinateTransformations.LinearMap(LDrawParser.LDRAW_BASE_FRAME)
SCALE = 0.01
LDrawParser.change_coordinate_system!(model,T_base,SCALE)

m = model.models["20009 - AT-TE Walker.mpd"]
id_generator = DuplicateIDGenerator{String}()
for step in m.steps
    for ref in step.lines
        global vis_root
        @show ref
        if !LDrawParser.has_part(model,ref.file)
            continue
        end
        p = model.parts[ref.file]
        name = id_generator(p.name)
        vec = LDrawParser.extract_surface_geometry(p)
        M = GeometryBasics.Mesh(coordinates(vec),faces(vec))
        setobject!(vis_root[name], M)
        tr = LDrawParser.build_transform(ref)
        settransform!(vis_root[name], tr)
    end
end
delete!(vis)


# construct model graph
model_graph = construct_assembly_graph(model)
model_tree = convert(GraphUtils.CustomNTree{GraphUtils._node_type(model_graph),String},model_graph)
# @assert maximum(map(v->indegree(model_tree,v),vertices(model_tree))) == 1
print(model_tree,v->summary(v.val),"\t")

sched = LDrawParser.construct_model_schedule(model)
model_spec = LDrawParser.extract_single_model(sched,"20009 - AT-TE Walker.mpd")

GraphUtils.validate_graph(model_spec)
