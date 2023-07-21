using Test
using CoordinateTransformations
using MeshCat

const CT = CoordinateTransformations

Revise.includet("/home/kylebrown/.julia/dev/ConstructionBots/src/potential_fields.jl")

# Cone potential
let
    p = ConePotential(-1.0,1.0,Point(0.0,0.0))
    x1 = Point(0.5,0.5)
    d1 = normalize(Point(-1.0,-1.0))

    y = p(x1)
    dx1 = potential_gradient(p,x1)
    @test isapprox(norm(dx1),1)
    @test isapprox(norm(dx1 - d1), 0.0, atol=1e-10)

    # out of range
    x2 = Point(2.0,2.0)
    dx2 = potential_gradient(p, x2)
    @test isapprox(norm(dx2),0)

end
# Rotational Potential
let
    p = ConePotential(-1.0,2.0,Point(0.0,0.0))
    rp = RotationalPotential(p,1.0,1.0)
    x = Point(1.0,1.0)
    potential_gradient(p,x)

    potential_gradient(rp,x)
    potential_gradient([rp],x)

end
# tent potential
let
    p = TentPotential(-1.0,1.0,Point(0.0,0.0),Point(0.0,2.0))
    x1 = Point(0.5,0.5)
    d1 = normalize(Point(-1.0,-0.0))
    dx1 = potential_gradient(p, x1)
    @test isapprox(norm(dx1),1)
    @test isapprox(norm(dx1 - d1), 0.0, atol=1e-10)

    # out of range
    x2 = Point(2.0,2.0)
    dx2 = potential_gradient(p, x2)
    @test isapprox(norm(dx2),0)

end
# BarrierPotential
let
    p = ConePotential(1.0,1.0,Point(0.0,0.0))
    pb = BarrierPotential(p,1.0,0.1)
    y = potential(pb,p.x)
    @test isapprox(y,pb.c/pb.z,atol=1e-10)

end
# superposed potentials
let
    p = [
        ConePotential(-1.0,1.0,Point(0.0,0.0)),
        TentPotential(-1.0,1.0,Point(0.0,0.0),Point(0.0,2.0))
    ]
    x1 = Point(0.5,0.5)
    d1 = normalize(Point(-1.0,-1.0)) + normalize(Point(-1.0,-0.0))
    dx1 = potential_gradient(p, x1)
    @test isapprox(norm(dx1 - d1), 0.0, atol=1e-10)

end

vis = MeshCat.Visualizer()
render(vis)

# demo where all robots are trying to get into the innermost circle, but only
# some are allowed to enter at that time
robots = Dict(
    1=>PotentialController(
        x = [8.0,0.0],
        circ_idx = 1,
        goal = [0.0,0.0],
        radius = ROBOT_RADIUS
    ),
    2=>PotentialController(
        x = [6.0,0.5],
        circ_idx = 2,
        goal = [0.0,0.0],
        radius = ROBOT_RADIUS
    ),
    3=>PotentialController(
        x = [6.0,-0.5],
        circ_idx = 3,
        goal = [0.0,0.0],
        radius = ROBOT_RADIUS
    ),
)

# a succession of circles that the robots are allowed to enter
circles = [
    j=>GeometryBasics.Circle(p,r) for (j,(p,r)) in enumerate([
        (Point(0.0,0.0),2.0),
        (Point(0.0,0.0),4.0),
        (Point(-1.75,1.0),6.0),
    ])
]
# to attract robots to their goal
goal_potentials = Dict(k=>ConePotential(1.0,Inf,r.goal) for (k,r) in robots)
# to repel robots from circles in which they do not belong
circle_potentials = Dict(j=>ConePotential(-1.0,c.r+ROBOT_RADIUS,c.center) for (j,c) in circles)
# for collision-avoidance between agents
collision_potentials = Dict(k=>ConePotential(-1.0,2*ROBOT_RADIUS,r.x) for (k,r) in robots)
# to push other agents out of the way
path_potentials = Dict(k=>TentPotential(-1.0,2*ROBOT_RADIUS,r.x,r.goal) for (k,r) in robots)

controller = GlobalPotentialController(
    robot_controllers = robots,
    goal_potentials = goal_potentials,
    circle_potentials = circle_potentials,
    collision_potentials = collision_potentials,
    path_potentials = path_potentials
)

update_potential_controllers!(controller)

# simulate

delete!(vis)
for (k,robot) in controller.robot_controllers
    setobject!(vis["goals"]["$k"],
        HyperSphere(Point(robot.goal...,0.0),0.25),
        MeshLambertMaterial(color=RGBA(0.0,1.0,0.0,0.5))
    )
    setobject!(vis["robots"]["$k"],
        Cylinder(Point(0.0,0.0,0.0), Point(0.0,0.0,0.1), ROBOT_RADIUS)
    )
    settransform!(vis["robots"]["$k"],CT.Translation(robot.x...,0.0))
end
for (k,circ) in circles
    setobject!(vis["circles"]["$k"],
        Cylinder(Point(circ.center...,0.0), Point(circ.center...,-0.1), circ.r),
        MeshLambertMaterial(color=RGBA(0.0,0.5,0.5,0.25))
    )
end

dt = 0.01
vmax = 1.0
for time_step in 1:1000
    # update controller position and path_potential simultaneuously
    update_potential_controllers!(controller)
    for (k,robot) in controller.robot_controllers
        # @show k
        # for p in robot.p
        #     if norm(potential_gradient(p,robot.x)) > 1e-4
        #         @show p, potential_gradient(p,robot.x)
        #     end
        # end
        # @show potential_gradient(robot.p,robot.x)
        # update positions
        robot.x[:] = robot.x .+ dt*robot.v
        settransform!(vis["robots"]["$k"],CT.Translation(robot.x...,0.0))
    end
    sleep(dt)
end
