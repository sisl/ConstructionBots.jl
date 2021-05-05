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
        (Point(-5,1.0),1.0),
        (Point(-2.0,-1.0),1.0),
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
    elseif norm(vec) < 0.5*vmax/dt
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
    # sleep(dt)
end
setanimation!(vis,anim.anim)