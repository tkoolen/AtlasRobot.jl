__precompile__()

module AtlasRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

packagepath() = joinpath(@__DIR__, "..", "deps")
urdfpath() = joinpath(packagepath(), "Atlas", "atlas.urdf")

const SOLE_TO_ANKLE_OFFSET = 0.07645

function flipsign_if_right(x::Number, side::Symbol)
    side == :left && return x
    side == :right && return -x
    error()
end

function mechanism(::Type{T} = Float64;
        floating = true,
        remove_fixed_tree_joints = true) where {T}
    RigidBodyDynamics.parse_urdf(urdfpath(); scalar_type=T, floating=floating, remove_fixed_tree_joints=remove_fixed_tree_joints)
end

function default_contact_force_model()
    normal_model = hunt_crossley_hertz(; k=500e3)
    tangential_model = ViscoelasticCoulombModel(0.8, 20e3, 100.)
    SplitContactForceModel(normal_model, tangential_model)
end

function add_sole_frames!(mechanism::Mechanism)
    soleframes = Dict{BodyID, CartesianFrame3D}()
    for side in (:left, :right)
        body = findbody(mechanism, "$(first(string(side)))_foot")
        bodyframe = default_frame(body)
        soleframe = soleframes[BodyID(body)] = CartesianFrame3D("$(body)_sole")
        sole_to_body = Transform3D(soleframe, bodyframe, SVector(0, 0, -SOLE_TO_ANKLE_OFFSET))
        add_frame!(body, sole_to_body)
    end
    return soleframes
end

function foot_contact_points(mechanism::Mechanism{T}) where T
    points = Dict{BodyID, Vector{Point3D{SVector{3, T}}}}()
    for side in (:left, :right)
        foot = findbody(mechanism, "$(first(string(side)))_foot")
        foot_points = points[BodyID(foot)] = Point3D{SVector{3, T}}[]
        frame = default_frame(foot)
        z = -SOLE_TO_ANKLE_OFFSET

        # heel
        push!(foot_points, Point3D(frame, -0.0876, flipsign_if_right(0.066, side), z))
        push!(foot_points, Point3D(frame, -0.0876, flipsign_if_right(-0.0626, side), z))

        # toe:
        push!(foot_points, Point3D(frame, 0.1728, flipsign_if_right(0.066, side), z))
        push!(foot_points, Point3D(frame, 0.1728, flipsign_if_right(-0.0626, side), z))

        # midfoot:
        push!(foot_points, Point3D(frame, 0.0426, flipsign_if_right(0.066, side), z))
        push!(foot_points, Point3D(frame, 0.0426, flipsign_if_right(-0.0626, side), z))
    end
    return points
end

function setnominal!(atlasstate::MechanismState)
    mechanism = atlasstate.mechanism
    zero!(atlasstate)
    kneebend = 1.1
    hipbendextra = 0.0
    left_shoulder_x_angle = -1.2
    elbow_y_angle = pi / 2
    left_elbow_x_angle = 0.2
    for sideprefix in ('l', 'r')
        knee = findjoint(mechanism, "$(sideprefix)_leg_kny")
        hippitch = findjoint(mechanism, "$(sideprefix)_leg_hpy")
        anklepitch = findjoint(mechanism, "$(sideprefix)_leg_aky")
        shoulder_x = findjoint(mechanism, "$(sideprefix)_arm_shx")
        elbow_y = findjoint(mechanism, "$(sideprefix)_arm_ely")
        elbow_x = findjoint(mechanism, "$(sideprefix)_arm_elx")
        set_configuration!(atlasstate, knee, kneebend)
        set_configuration!(atlasstate, hippitch, -kneebend / 2 + hipbendextra)
        set_configuration!(atlasstate, anklepitch, -kneebend / 2 - hipbendextra)
        set_configuration!(atlasstate, shoulder_x, sideprefix == 'l' ? left_shoulder_x_angle : -left_shoulder_x_angle)
        set_configuration!(atlasstate, elbow_y, elbow_y_angle)
        set_configuration!(atlasstate, elbow_x, sideprefix == 'l' ? left_elbow_x_angle : -left_elbow_x_angle)
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
