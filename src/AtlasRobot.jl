__precompile__()

module AtlasRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

packagepath() = joinpath(@__DIR__, "..", "deps")
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
        remove_fixed_tree_joints = true, add_flat_ground=false) where {T}
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

    if add_flat_ground
        frame = root_frame(mechanism)
        ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
        add_environment_primitive!(mechanism, ground)
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end

function setnominal!(atlasstate::MechanismState)
    mechanism = atlasstate.mechanism
    zero!(atlasstate)
    kneebend = 1.1
    hipbendextra = 0.1
    for sideprefix in ('l', 'r')
        knee = findjoint(mechanism, "$(sideprefix)_leg_kny")
        hippitch = findjoint(mechanism, "$(sideprefix)_leg_hpy")
        anklepitch = findjoint(mechanism, "$(sideprefix)_leg_aky")
        set_configuration!(atlasstate, knee, [kneebend])
        set_configuration!(atlasstate, hippitch, [-kneebend / 2 + hipbendextra])
        set_configuration!(atlasstate, anklepitch, [-kneebend / 2 - hipbendextra])
    end
    floatingjoint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(atlasstate, floatingjoint, [1; 0; 0; 0; 0; 0; 0.85])
    atlasstate
end

function __init__()
    if !isfile(urdfpath())
        error("Could not find $(urdfpath()). Please run `import Pkg; Pkg.build(\"AtlasRobot\")`.")
    end
end

end # module
