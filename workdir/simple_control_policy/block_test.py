import Sofa
import SofaRuntime
import sys
import os
#from stlib3.physics.constraints import FixedBox
from stlib3.scene import Scene
from splib.animation import AnimationManager, addAnimation
#from elasticmaterialobject import ElasticMaterialObject
#from stlib3.physics.deformable.elasticmaterialobject import ElasticMaterialObject
#from softrobots.actuators import PneumaticCavity
import time
import math
import numpy as np
import timeit

path = os.path.dirname(os.path.abspath(__file__)) + '/'
meshpath = path+"mesh/"

# scene helper function, defining materials and constraints
def createSceneReal(rootNode, dt):

    
    # this sets gravity and dt in the simulator

    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 0], dt=dt)
    # marks a required plugin and some visual style stuff

    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.addObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    # animation loop used for legrangian constraints
    rootNode.addObject('FreeMotionAnimationLoop')
    # linear solver (with parameters from example file)
    rootNode.addObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    # set color of background, just visual style
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    # YM of the material. All in kg / m / s
    volumeMeshFileName=meshpath+"two_cell_robot_solid.msh"
    name="two_cell_robot"
    rotation=[0.0, 0.0, 0.0]
    translation=[0.0, 0.0, 0.0]
    scale=[1.0, 1.0, 1.0]
    surfaceMeshFileName=None
    collisionMesh=None
    withConstrain=True
    surfaceColor=[1.0, 1.0, 1.0]
    poissonRatio=0.3
    youngModulus=1800
    totalMass=1.0
    solver=None
    # elestic material from prefab
    '''two_cell_robot = ElasticMaterialObject(name="two_cell_robot",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+"two_cell_robot_solid.msh",
                                  surfaceMeshFileName=None,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation=None,
                                  rotaiton=None,
                                  scale=[1.,1.0,1.0],
                                  collisionMesh=None,
                                  surfaceColor=None)

    '''
    
    two_cell_robot = rootNode.createChild( 'two_cell_robot' )
    two_cell_robot.addObject('MeshGmshLoader', name='loader', filename=volumeMeshFileName, rotation=rotation, translation=translation, scale3d=scale)
    two_cell_robot.addObject( 'EulerImplicitSolver', name='integration')
    two_cell_robot.addObject( 'SparseLDLSolver', name="solver")
    two_cell_robot.addObject( 'TetrahedronSetTopologyContainer', src="@loader", name="container")
    two_cell_robot.addObject( 'MechanicalObject', template='Vec3d',  name='dofs') #topology="@container",
    two_cell_robot.addObject( 'UniformMass', totalMass=totalMass, name='mass')
    two_cell_robot.addObject( 'TetrahedronFEMForceField', template='Vec3d', method='large', name='forcefield', poissonRatio=poissonRatio, youngModulus=youngModulus)

    two_cell_robot.addObject( 'LinearSolverConstraintCorrection', template='Vec3d', solverName='solver')
    
    #two_cell_robot.addObject( 'FixedConstraint', template='Vec3d', topology="@container" )


    #vm = two_cell_robot.createChild( 'VisualModel' )
    #vm.addObject( 'MeshSTLLoader', template='')
    #vm.addObject( 'OglModel', template='Vec3d', topology="@/two_cell_robot/container" )
    #vm.addObject( 'BarycentricMapping', template='Vec3d,Vec3d', input="@../" , output="@./" )

    # impose the constraints TODO: get these constraints so I can add gravity.
    #two_cell_robot.addObject("FixedConstraint", indices=fixed_const_str)

    # create the two actuators
    cavity1 = two_cell_robot.createChild('cavity1')
    cavity1.addObject('MeshSTLLoader', name='loader', filename=meshpath + "two_cell_robot_cell1.stl",
                        translation="0 0 0")
    cavity1.addObject('Mesh', src='@loader', name='topo')
    cavity1.addObject('MechanicalObject', name='cavity')
    cavity1.addObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.6001",
                        triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
    cavity1.addObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')

    cavity2 = two_cell_robot.createChild('cavity2')
    cavity2.addObject('MeshSTLLoader', name='loader', filename=meshpath + "two_cell_robot_cell2.stl",
                        translation="0 0 0")
    cavity2.addObject('Mesh', src='@loader', name='topo')
    cavity2.addObject('MechanicalObject', name='cavity')
    cavity2.addObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001",
                        triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
    cavity2.addObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')

    # two_cell_robot visualization
    two_cell_robotVisu = two_cell_robot.createChild('visu')
    two_cell_robotVisu.addObject('TriangleSetTopologyContainer', name='container')
    two_cell_robotVisu.addObject('TriangleSetTopologyModifier')
    #two_cell_robotVisu.addObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    two_cell_robotVisu.addObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    two_cell_robotVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    two_cell_robotVisu.addObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=translation)
    two_cell_robotVisu.addObject('IdentityMapping')
    
    return two_cell_robot


# "Main" function that runSofa uses to build the scene.
def createScene(rootNode):
    dt = 0.001 # set the time step for the simulator
    # set length scale

    '''
    # information about the output.
    info_arr = np.array([float(length_scale), 0, dt, num_nodes])

    print "info array ", info_arr
    print "number of nodes", num_nodes, " num end nodes ", len(fixed_const_lst), " num mid nodes ", len(middle_nodes_lst)
    '''

    # simulation timer
    start = timeit.default_timer()

    #animation function called at each step
    def my_animation(target, factor):
        factor = factor*2*np.pi
        pressureValue1 = target["two_cell_robot.cavity1.SurfacePressureConstraint.value"].getValueString()
        print(pressureValue1)
        pressureValue1 = float(pressureValue1)
        target.two_cell_robot.cavity1.SurfacePressureConstraint.findData('value').value = str(0.1)
        print(dir(target.two_cell_robot.cavity1.SurfacePressureConstraint))

        
        print(target.two_cell_robot.cavity1.SurfacePressureConstraint.pressure.value)
        target.two_cell_robot.cavity1.SurfacePressureConstraint.findData('value').value = str(10.0)


        print(factor)

    # the exit func is called when duration of time has elapsed, it marks the simulation time
    # saves it, and saves the collected data to an array. And exits the simulation with sys exit
    def ExitFunc(target, factor):
        # save the various data.
        runtime = timeit.default_timer() - start
        print("runtime", runtime)
        sys.exit(0)

    def getObject(self, name):
        return name
    # this is the Sofa animation function we pass it our animation function
    # and along with the exit function.
    createSceneReal(rootNode, dt)
    
    AnimationManager(rootNode)
    addAnimation(my_animation, {"target": rootNode}, duration=2.0, mode="once", onDone=ExitFunc)

    return rootNode
    

    
