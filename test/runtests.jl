using AtlasRobot
using MechanismGeometries
using Compat.Test

@testset "load geometries" begin
    mechanism = AtlasRobot.mechanism()
    visuals = URDFVisuals(AtlasRobot.urdfpath(); package_path = [AtlasRobot.packagepath()])
    meshgeometry = visual_elements(mechanism, visuals)
    @test length(meshgeometry) == 48
end
