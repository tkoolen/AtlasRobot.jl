using AtlasRobot
using Compat.Test

import RigidBodyDynamics: num_velocities

@testset "atlas" begin
    mechanism = AtlasRobot.mechanism()
    @test num_velocities(mechanism) == 36
    meshdir = joinpath(AtlasRobot.packagepath(), "Atlas", "urdf", "meshes")
    @test isdir(meshdir)
    @test count(x -> endswith(x, ".obj"), readdir(meshdir)) ==45
end
