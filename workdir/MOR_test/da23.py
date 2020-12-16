import Sofa


def createScene(root):
    root.dt = 0.001
    root.gravity = [0,-0,9.8]


    root.createObject( 'VisualStyle', template='')
    root.createObject( 'PythonScriptController', template='')
    root.createObject( 'RequiredPlugin', template='')
    root.createObject( 'VisualStyle', template='')
    root.createObject( 'FreeMotionAnimationLoop', template='')
    root.createObject( 'GenericConstraintSolver', template='')
    root.createObject( 'BackgroundSetting', template='')
    root.createObject( 'RequiredPlugin', template='')
    root.createObject( 'DefaultVisualManagerLoop', template='')
    root.createObject( 'InteractiveCamera', template='')


    Config_Node2 = root.createChild( 'Config' )
    Config_Node2.createObject( 'RequiredPlugin', template='')
    Config_Node2.createObject( 'RequiredPlugin', template='')
    Config_Node2.createObject( 'OglSceneFrame', template='')


    disk_Node3 = root.createChild( 'disk' )
    disk_Node3.createObject( 'MeshGmshLoader', template='')
    disk_Node3.createObject( 'EulerImplicitSolver', template='')
    disk_Node3.createObject( 'SparseLDLSolver', template='CompressedRowSparseMatrixd')
    disk_Node3.createObject( 'TetrahedronSetTopologyContainer', template='')
    disk_Node3.createObject( 'MechanicalObject', template='Vec3d', topology="@container" )
    disk_Node3.createObject( 'UniformMass', template='Vec3d,double')
    disk_Node3.createObject( 'TetrahedronFEMForceField', template='Vec3d', topology="@container" )
    disk_Node3.createObject( 'LinearSolverConstraintCorrection', template='Vec3d')
    disk_Node3.createObject( 'FixedConstraint', template='Vec3d', topology="@container" )


    VisualModel_Node4 = disk_Node3.createChild( 'VisualModel' )
    VisualModel_Node4.createObject( 'MeshSTLLoader', template='')
    VisualModel_Node4.createObject( 'OglModel', template='Vec3d', topology="@/disk/container" )
    VisualModel_Node4.createObject( 'BarycentricMapping', template='Vec3d,Vec3d', input="@../" , output="@./" )


    cavity_Node5 = disk_Node3.createChild( 'cavity' )
    cavity_Node5.createObject( 'MeshSTLLoader', template='')
    cavity_Node5.createObject( 'MeshTopology', template='')
    cavity_Node5.createObject( 'MechanicalObject', template='Vec3d', topology="@topo" )
    cavity_Node5.createObject( 'SurfacePressureConstraint', template='Vec3d')
    cavity_Node5.createObject( 'BarycentricMapping', template='Vec3d,Vec3d', input="@../" , output="@./" )


    visu_Node6 = disk_Node3.createChild( 'visu' )
    visu_Node6.createObject( 'TriangleSetTopologyContainer', template='')
    visu_Node6.createObject( 'TriangleSetTopologyModifier', template='')
    visu_Node6.createObject( 'TriangleSetTopologyAlgorithms', template='Vec3d')
    visu_Node6.createObject( 'TriangleSetGeometryAlgorithms', template='Vec3d', topology="@container" )
    visu_Node6.createObject( 'Tetra2TriangleTopologicalMapping', template='', input="@../container" , output="@container" )
    visu_Node6.createObject( 'OglModel', template='Vec3d', topology="@container" )
    visu_Node6.createObject( 'IdentityMapping', template='Vec3d,Vec3d', input="@../" , output="@./" )
