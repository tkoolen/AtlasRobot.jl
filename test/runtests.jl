using AtlasRobot
using RigidBodyTreeInspector
using Base.Test

@testset "load geometries" begin
    mechanism = AtlasRobot.mechanism()
    geometries = RigidBodyTreeInspector.parse_urdf(AtlasRobot.urdfpath(), mechanism; package_path = [AtlasRobot.packagepath()]);
    @test length(geometries) == 48
end
