using ConstructionBots
using HierarchicalGeometry
using GeometryBasics
using GraphUtils
using LinearAlgebra
using MeshCat
using CoordinateTransformations
using TaskGraphs
Revise.includet(joinpath(pathof(ConstructionBots),"..","render_tools.jl"))

const CT = CoordinateTransformations

vis = MeshCat.Visualizer()
MeshCat.render(vis)

agent_radius = 0.5
pos = [-8.0,0.0]
goal = [8.0,0.0]
circles = [
    j=>GeometryBasics.Circle(p,r) for (j,(p,r)) in enumerate([
        # (Point(0.0,0.0),2.5),
        # (Point(-5.0,-2.25),1.5),
        # (Point(5.0,-2.5),1.5),
        (Point(-7.9,0.0),1.0),
        (Point(-5,3.0),1.0),
        (Point(-2.0,-1.0),1.0),
        (Point(-2.0,-3.75),1.0),
        (Point(2.0,1.0),1.0),
        (Point(5.0,-1.0),1.0),
    ])
]
policy = ConstructionBots.TangentBugPolicy(
    vmax = 1.0,
    dt = 0.05,
    agent_radius=agent_radius,
)

delete!(vis)
setobject!(vis["goal"],
    HyperSphere(Point(0.0,0.0,0.0),0.25),
    MeshLambertMaterial(color=RGBA(0.0,1.0,0.0,0.5))
)
setobject!(vis["waypoint"],
    HyperSphere(Point(0.0,0.0,0.0),0.25),
    MeshLambertMaterial(color=RGBA(1.0,0.0,0.0,0.5))
)
setobject!(vis["robot"],
    Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,0.1), agent_radius)
)
settransform!(vis["robot"],CT.Translation(pos...,0.0))
settransform!(vis["goal"],CT.Translation(goal...,0.0))
settransform!(vis["waypoint"],CT.Translation(goal...,0.0))
for (k,circ) in circles
    setobject!(vis["circles"]["$k"],
        Cylinder(Point(circ.center...,0.0), Point(circ.center...,-0.1), circ.r),
        MeshLambertMaterial(color=RGBA(0.0,0.5,0.5,0.25))
    )
end

# simulate
anim = AnimationWrapper(0)
dt = policy.dt
vmax = policy.vmax
for time_step in 1:800
    waypoint = ConstructionBots.tangent_bug_policy!(
            policy,circles,pos,goal
        )
    vec = waypoint - pos
    if norm(vec) > vmax*dt
        vel = vmax * normalize(vec)
    elseif policy.mode == :MOVE_TOWARD_GOAL && norm(vec) < 0.5*vmax/dt
        println("FINISHED")
        break
    else
        vel = vec / dt
    end
    pos = pos + dt*vel
    # @show time_step, pos
    atframe(anim,current_frame(anim)) do
        settransform!(vis["waypoint"],CT.Translation(waypoint...,0.0))
        settransform!(vis["robot"],CT.Translation(pos...,0.0))
    end
    step_animation!(anim)
end
setanimation!(vis,anim.anim)


# # Test circle_avoidance_policy
# circles = [
#     LazySets.Ball2(SVector(0.0,0.0),1.0),
#     # LazySets.Ball2(SVector(1.0,-3.0),1.0),
#     LazySets.Ball2(SVector(0.0,4.0),2.0),
#     ]
# goal = [0.0,8.0]
# pos =  [0.0,-6.0]
# agent_radius = HG.default_robot_radius()
# dt = 0.025
# vmax = 1.0
# delete!(vis)
# setobject!(vis[:robot],default_robot_geom(),MeshLambertMaterial(color=RGB(0.1,0.1,0.1)))
# settransform!(vis[:robot],CT.Translation(pos...,0.0))
# setobject!(vis[:goal],default_robot_geom(),MeshLambertMaterial(color=RGBA(0.0,1.0,0.0,0.2)))
# settransform!(vis[:goal],CT.Translation(goal...,0.0))
# for (i,c) in enumerate(circles)
#     setobject!(vis[:circles][string(i)],convert(GeometryBasics.Sphere,HG.project_to_3d(c)),
#      MeshLambertMaterial(color=RGBA(1.0,0.0,0.0,0.1)))
# end
# for t in 1:1000
#     goal_pt = ConstructionBots.circle_avoidance_policy(circles,agent_radius,pos,goal;buffer=0.5)
#     vel = normalize(goal_pt - pos) * min(vmax,norm(goal_pt - pos)/dt)
#     if any(isnan,vel)
#         vel = [0.0,0.0]
#         break
#     end
#     pos = pos .+ vel*dt
#     # update visualizer
#     settransform!(vis[:robot],CT.Translation(pos...,0.0))
# end
