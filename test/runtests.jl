using Test
using AtlasRobot
using RigidBodyDynamics
using RigidBodyDynamics.Contact: location

@testset "atlas" begin
    mechanism = AtlasRobot.mechanism()
    @test num_velocities(mechanism) == 36
    meshdir = joinpath(AtlasRobot.packagepath(), "Atlas", "urdf", "meshes")
    @test isdir(meshdir)
    @test count(x -> endswith(x, ".obj"), readdir(meshdir)) == 45
end

@testset "setnominal!" begin
    mechanism = AtlasRobot.mechanism(add_flat_ground=true)
    state = MechanismState(mechanism)
    AtlasRobot.setnominal!(state)
    for sideprefix in ('l', 'r')
        foot = findbody(mechanism, "$(sideprefix)_foot")
        for point in contact_points(foot)
            contact_location_world = transform(state, location(point), root_frame(mechanism))
            @test contact_location_world.v[3] ≈ 0 atol=2e-2
        end
    end
    @test center_of_mass(state).v[3] > 1
end
