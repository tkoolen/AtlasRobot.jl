__precompile__()

module AtlasRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

packagepath() = joinpath(dirname(@__DIR__), "deps")
urdfpath() = joinpath(packagepath(), "Atlas", "atlas.urdf")

function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end

function flipsign_if_right(x::Number, side::Symbol)
    side == :left && return x
    side == :right && return -x
    error()
end

function mechanism(::Type{T} = Float64;
        floating = true,
        contactmodel = default_contact_model(),
        remove_fixed_tree_joints = true) where {T}
    mechanism = RigidBodyDynamics.parse_urdf(T, urdfpath())

    if floating
        pelvis = findbody(mechanism, "pelvis")
        basejoint = joint_to_parent(pelvis, mechanism)
        floatingjoint = Joint(basejoint.name, frame_before(basejoint), frame_after(basejoint), QuaternionFloating{T}())
        replace_joint!(mechanism, basejoint, floatingjoint)
        basejoint = floatingjoint
    end

    if contactmodel != nothing
        for side in (:left, :right)
            foot = findbody(mechanism, "$(first(string(side)))_foot")
            frame = default_frame(foot)
            z = -0.07645

            # heel
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0876, flipsign_if_right(0.066, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0876, flipsign_if_right(-0.0626, side), z), contactmodel))

            # toe:
            add_contact_point!(foot, ContactPoint(Point3D(frame, 0.1728, flipsign_if_right(0.066, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, 0.1728, flipsign_if_right(-0.0626, side), z), contactmodel))

            # midfoot:
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0426, flipsign_if_right(0.066, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.0426, flipsign_if_right(-0.0626, side), z), contactmodel))
        end
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end

end # module
