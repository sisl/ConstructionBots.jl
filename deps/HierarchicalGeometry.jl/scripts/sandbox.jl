using HierarchicalGeometry
using LazySets
using Polyhedra
using LightGraphs, GraphUtils, Test
using GeometryBasics
using CoordinateTransformations
using Rotations
using StaticArrays
using MeshCat
using LinearAlgebra
# using Meshing
using Colors
using Test
using Plots


ball = LazySets.Ball2(zeros(3),2.0)

bbox = overapproximate(ball)
hpoly = convert(LazySets.HPolytope,bbox)
overapproximate(hpoly,Hyperrectangle)
overapproximate(hpoly,LazySets.BallInf)
vpoly = tovrep(hpoly)
poly = Polyhedra.polyhedron(hpoly)

translated_hpoly = LazySets.Translation(hpoly,[1.0,1.0,1.0])

ball2 = LazySets.Ball2([3.0,0.0,0.0],1.0)
lazy_set = UnionSet(ball,ball2)
model = HierarchicalGeometry.equatorial_overapprox_model()
hpoly = overapproximate(lazy_set,model)
poly = Polyhedra.polyhedron(hpoly)
# poly = Polyhedra.polyhedron(LazySets.translate(hpoly,[3.0,0.0,0.0]))

# to 2D for assigning robot carry positions
geom = hpoly
polygon = VPolygon(convex_hull(map(v->v[1:2],vertices_list(hpoly))))

transport_model = (
    robot_radius = 1.0,
    max_area_per_robot = 10.0,
    max_volume_per_robot = 30.0,
)
support_pts = HierarchicalGeometry.select_support_locations(geom,transport_model)

plt = plot(polygon, 1e-3, aspectratio=1, alpha=0.4)
for pt in support_pts
    circ = Ball2(pt,transport_model.robot_radius)
    plot!(plt, circ)
end
display(plt)

# POLYHEDRON_MATERIAL = MeshPhongMaterial(color=RGBA{Float32}(1, 0, 0, 0.5))
# vis = Visualizer()
# render(vis)
# setobject!(vis[:ball], GeometryBasics.Sphere(GeometryBasics.Point(ball.center...),ball.radius))
# setobject!(vis[:ball2], GeometryBasics.Sphere(GeometryBasics.Point(ball2.center...),ball2.radius))
# setobject!(vis[:polytope], Polyhedra.Mesh(poly), POLYHEDRON_MATERIAL)
# setobject!(vis[:polytope], Polyhedra.Mesh{3}(poly), POLYHEDRON_MATERIAL)
#
# MeshCat.delete!(vis)
