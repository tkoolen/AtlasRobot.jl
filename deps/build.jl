datadir = "atlas"
atlas_examples_url = "https://raw.githubusercontent.com/RobotLocomotion/drake/73a8da32cd41ff7fd023c3680f8250860cbd0e6b/examples/atlas/"
urdfpath = "urdf/atlas_convex_hull.urdf"

meshpaths = ["urdf/meshes/GRIPPER_OPEN_chull.obj";
    "urdf/meshes/head.obj";
    "urdf/meshes/head_camera.obj";
    "urdf/meshes/head_camera_chull.obj";
    "urdf/meshes/head_chull.obj";
    "urdf/meshes/l_foot.obj";
    "urdf/meshes/l_foot_chull.obj";
    "urdf/meshes/l_lglut.obj";
    "urdf/meshes/l_lglut_chull.obj";
    "urdf/meshes/l_lleg.obj";
    "urdf/meshes/l_lleg_chull.obj";
    "urdf/meshes/l_talus.obj";
    "urdf/meshes/l_uglut.obj";
    "urdf/meshes/l_uglut_chull.obj";
    "urdf/meshes/l_uleg.obj";
    "urdf/meshes/l_uleg_chull.obj";
    "urdf/meshes/ltorso.obj";
    "urdf/meshes/mtorso.obj";
    "urdf/meshes/pelvis.obj";
    "urdf/meshes/pelvis_chull.obj";
    "urdf/meshes/r_clav.obj";
    "urdf/meshes/r_clav_chull.obj";
    "urdf/meshes/r_farm.obj";
    "urdf/meshes/r_farm_chull.obj";
    "urdf/meshes/r_foot.obj";
    "urdf/meshes/r_foot_chull.obj";
    "urdf/meshes/r_hand.obj";
    "urdf/meshes/r_hand_chull.obj";
    "urdf/meshes/r_larm.obj";
    "urdf/meshes/r_larm_chull.obj";
    "urdf/meshes/r_lglut.obj";
    "urdf/meshes/r_lglut_chull.obj";
    "urdf/meshes/r_lleg.obj";
    "urdf/meshes/r_lleg_chull.obj";
    "urdf/meshes/r_scap.obj";
    "urdf/meshes/r_scap_chull.obj";
    "urdf/meshes/r_talus.obj";
    "urdf/meshes/r_uarm.obj";
    "urdf/meshes/r_uarm_chull.obj";
    "urdf/meshes/r_uglut.obj";
    "urdf/meshes/r_uglut_chull.obj";
    "urdf/meshes/r_uleg.obj";
    "urdf/meshes/r_uleg_chull.obj";
    "urdf/meshes/utorso.obj";
    "urdf/meshes/utorso_chull.obj"]

ispath(datadir) || mkpath(datadir)
download(atlas_examples_url * urdfpath, joinpath(datadir, "atlas.urdf"))

for meshpath in meshpaths
    meshdir, meshfilename = splitdir(meshpath)
    meshdir = joinpath(datadir, meshdir)
    ispath(meshdir) || mkpath(meshdir)
    download(atlas_examples_url * meshpath, joinpath(datadir, meshpath))
end
