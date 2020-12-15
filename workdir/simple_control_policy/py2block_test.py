import Sofa
import sys
import os
#from stlib3.physics.constraints import FixedBox
from stlib.scene import Scene
from splib.animation import animate
from stlib.physics.deformable.elasticmaterialobject import ElasticMaterialObject
#from softrobots.actuators import PneumaticCavity
import time
import math
#import numpy as np
import timeit

path = os.path.dirname(os.path.abspath(__file__)) + '/'
meshpath = path+"mesh/"

# scene helper function, defining materials and constraints
def createSceneReal(rootNode, dt):

    # this sets gravity and dt in the simulator

    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 0], dt=dt)
    # marks a required plugin and some visual style stuff

    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    # animation loop used for legrangian constraints
    rootNode.createObject('FreeMotionAnimationLoop')
    # linear solver (with parameters from example file)
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    # set color of background, just visual style
    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    # YM of the material. All in kg / m / s
    YoungModulus = 1800
    Translation = None
    # elestic material from prefab
    two_cell_robot = ElasticMaterialObject(name="two_cell_robot",
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
    

    # impose the constraints TODO: get these constraints so I can add gravity.
    #two_cell_robot.createObject("FixedConstraint", indices=fixed_const_str)

    # create the two actuators
    cavity1 = two_cell_robot.createChild('cavity1')
    cavity1.createObject('MeshSTLLoader', name='loader', filename=meshpath + "two_cell_robot_cell1.stl",
                        translation="0 0 0")
    cavity1.createObject('Mesh', src='@loader', name='topo')
    cavity1.createObject('MechanicalObject', name='cavity')
    cavity1.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001",
                        triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
    cavity1.createObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')

    cavity2 = two_cell_robot.createChild('cavity2')
    cavity2.createObject('MeshSTLLoader', name='loader', filename=meshpath + "two_cell_robot_cell2.stl",
                        translation="0 0 0")
    cavity2.createObject('Mesh', src='@loader', name='topo')
    cavity2.createObject('MechanicalObject', name='cavity')
    cavity2.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001",
                        triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
    cavity2.createObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')

    # two_cell_robot visualization
    two_cell_robotVisu = two_cell_robot.createChild('visu')
    two_cell_robotVisu.createObject('TriangleSetTopologyContainer', name='container')
    two_cell_robotVisu.createObject('TriangleSetTopologyModifier')
    two_cell_robotVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    two_cell_robotVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    two_cell_robotVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    two_cell_robotVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    two_cell_robotVisu.createObject('IdentityMapping')
    
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
    def animation(target, factor):
        factor = factor*2*np.pi
        pressureValue1 = target.two_cell_robot.cavity1.SurfacePressureConstraint.findData('value').value[0][0] + 0.01
        pressureValue1 = np.sin(factor)*0.6

        target.two_cell_robot.cavity1.SurfacePressureConstraint.findData('value').value = str(pressureValue1)

        pressureValue2 = target.two_cell_robot.cavity2.SurfacePressureConstraint.findData('value').value[0][0] + 0.01
        pressureValue2 = np.cos(factor)*0.6

        target.two_cell_robot.cavity2.SurfacePressureConstraint.findData('value').value = str(pressureValue2)
        time.sleep(4)
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
    animate(animation, {"target": rootNode}, duration=2.0, mode="once", onDone=ExitFunc)

    return rootNode
    

    
