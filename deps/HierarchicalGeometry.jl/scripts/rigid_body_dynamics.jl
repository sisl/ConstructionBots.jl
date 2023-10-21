using RigidBodyDynamics
using LinearAlgebra
using StaticArrays
using Rotations
using CoordinateTransformations

FLOATING_JOINT_TYPE = QuaternionFloating{Float64}

g = -9.81
world = RigidBody{Float64}("world")

m = Mechanism(world;gravity=SVector(0,0,g))

robot1 = RigidBody{Float64}("robot1")
robot1_joint = Joint("robot1_joint", FLOATING_JOINT_TYPE())
r1_to_world = one(Transform3D,frame_before(robot1_joint),default_frame(world))
attach!(m, world, robot1, robot1_joint;joint_pose=r1_to_world)

"""
    quat_floating_config(t::CoordinateTransformations.AffineMap)

converts transform `t` into a 7-dimensional configuration vector to define the
configuration of a `QuaternionFloating{Float64}` joint.
    `v[1:3] == t.translation`
    `v[4:7] == q.w,q.x,q.y,q.z` for quaternion `q` representing the rotation
    part of the transform
"""
function quat_floating_config(t::CoordinateTransformations.AffineMap)
    q = Rotations.UnitQuaternion(t.linear)
    p = t.translation
    return SVector(p[1],p[2],p[3],q.w,q.x,q.y,q.z)
end
function Base.convert(::Type{CoordinateTransformations.AffineMap},t::RigidBodyDynamics.Transform3D)
    q = UnitQuaternion(t.mat[1:3,1:3])
    t = CoordinateTransformations.Translation(t.mat[1:3,4]...)
    return t ∘ CoordinateTransformations.LinearMap(q)
end

state = MechanismState(m)
quat = Rotations.UnitQuaternion(RotZ(π/4))
p = CoordinateTransformations.Translation(0.0, 0.0, 0.0)
t = p ∘ CoordinateTransformations.LinearMap(quat)
q = quat_floating_config(t)
set_configuration!(state,robot1_joint,q)
v = SVector(
    0.0, 0.0, 0.0, # linear velocity
    0.0, 0.0, π/2 # angular velocity
)
set_velocity!(state,robot1_joint,v)
# Now how to deal with integration? Since I'm limiting the action space, the integration
# can just be Euler integration for the most part. Right?
struct EulerIntegrator{S,A}
    state::S
    action::A
end

function integrate_euler()
end
