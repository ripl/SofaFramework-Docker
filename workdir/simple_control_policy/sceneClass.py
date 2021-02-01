# encoding: utf-8
#!/usr/bin/python3

from __future__ import print_function
import sys

python2_paths = ['', '/home/sofauser/workdir/simple_control_policy', '/builds/build/master/lib/python/site-packages', '/builds/plugins/ModelOrderReduction/python', '/builds/src/tools/sofa-launcher', '/builds/plugins/STLIB/python', '/builds/plugins/SoftRobots/python', '/pkgs', '/builds/plugins/STLIB/python/splib/animation',]# '/usr/lib/python2.7', '/usr/lib/python2.7', '/usr/lib/python2.7/plat-x86_64-linux-gnu', '/usr/lib/python2.7/lib-tk', '/usr/lib/python2.7/lib-old', '/usr/lib/python2.7/lib-dynload', '/usr/local/lib/python2.7/dist-packages', '/usr/lib/python2.7/dist-packages']
for i in python2_paths:
    sys.path.insert(1, i)


import os
#from stlib3.scene import Scene
import numpy as np
import timeit
from splib.animation import AnimationManager
from splib.animation import animate as addAnimation






class SceneDefinition:
    """This class handles the scene creation of the robot model"""
    def __init__(self, rootNode, design=np.zeros([1,1,1]), meshFolder="",
                 poissonRatio=0.3, youngModulus= 18000, totalMass = 1.0,
                 dt=0.001, with_gui=False, debug=False, collision=False, constraint=""):
        """initialize the class"""
        print("__init__ called")
        properties = [
            {'name': 'rootNode', 'type': 'sofa node', 'help': 'root node', 'default': ''},
            {'name': 'meshFolder', 'type': 'string', 'help': 'Path to volume mesh and cavity files', 'default': ''},
            {'name': 'design', 'type': 'ndarray', 'help': 'numpy array specifying design, coords -1 solid, 0 cavity', 'default': 'np.zeros([1,1,1])'},
            {'name': 'dt', 'type': 'double', 'help': 'time step', 'default': 0.001},
            {'name': 'poissonRatio', 'type': 'double', 'help': 'Poisson ratio', 'default': 0.3},
            {'name': 'youngModulus', 'type': 'double', 'help': "Young's modulus", 'default': 18000.0},
            {'name': 'totalMass', 'type': 'double', 'help': 'Total mass', 'default': 1.0}]
    
        # design as described in the sim wrapper
        self.design = design
        self.rootNode = rootNode
        self.meshFolder = meshFolder
        self.poissonRatio = poissonRatio
        self.youngModulus = youngModulus
        self.totalMass = totalMass
        self.with_gui = with_gui
        self.debug = debug
        self.with_collision = collision


        # this is the position and orientation of the robot. It is not fully
        # implemented so I think it is bedst to leave it as is.
        self.rotation = [0.0, 0.0, 0.0]
        self.translation = [0.0, 0.0, 0.0]
        
        # this will be the base of the robot in the scene graph
        self.volume = None

        # these two lines replace:
        # from stlib3.scene import Scene
        # Scene(self.rootNode, gravity=[0, 0.0, 0], dt=dt)
        # to remove the dependency
        self.rootNode.findData('gravity').value = [0.0, 0, -9.8*10**4]
        self.rootNode.findData('dt').value = dt
        

        full_required_plugins = ['SofaTopologyMapping', 'SofaOpenglVisual', 'SofaSparseSolver', 'SofaConstraint',
                            'SofaGeneralLoader', 'SoftRobots', 'SofaBoundaryCondition', 'SofaMiscCollision']
        
        required_plugins = ['SofaOpenglVisual', 'SofaSparseSolver', 
                            'SoftRobots',  'SofaMiscCollision']
        for i in required_plugins:
            print(i)
            self.rootNode.createObject("RequiredPlugin", name='req_p' + i, pluginName=i)
        print("req plugins?")
        # create all the material and cavities
        self.place_materials_and_cavities_and_solvers() #testing MOR
        #self.mor_test()
        
        # fix the base (all nodes where z=0)
        if "set_z" in constraint:
            self.fix_z()
        if "floor" in constraint:
            self.add_floor()

    def add_floor(self):
        print("add_floor called")
        #Creating the floor

        planeNode = self.rootNode.createChild('Plane')
        planeNode.createObject('MeshObjLoader', name='loader', filename="mesh/floorFlat.obj", triangulate="true")
        planeNode.createObject('Mesh', src="@loader")
        planeNode.createObject('MechanicalObject', src="@loader", rotation="90 0 0", translation="0 35 -1", scale="15")
        planeNode.createObject('TriangleCollisionModel', moving=False, simulated=False, group="1")
        planeNode.createObject('LineCollisionModel', moving=False, simulated=False, group="1")
        planeNode.createObject('PointCollisionModel', moving=False, simulated=False, group="1")
        if self.with_gui:
            planeNode.createObject('OglModel', name="Visual", src="@loader", color="1 0 0 1",
                                   rotation="90 0 0", translation="0 35 -1", scale="15")

    def fix_z(self):
        '''Fix all vertexes in mesh where z=0, this is so that one the robot doesn't
        fly off to infinity if it gets pushed a little bit, and two so it has something
        to push against while for now: it does simple shape rewards'''
        print("fix_z called")
        vertexes = self.rootNode.volume.loader.getData('position').value #.array()
        fix_string = " "
        for i, pos in enumerate(vertexes):
            if -0.01 <= pos[2] <= 0.01:
                # print(i,pos)
                
                fix_string = fix_string + " " + str(i)
        # print("FIXED POINTS", fix_string)
        # this adds a constraint that fixes these vertexes
        self.volume.createObject("FixedConstraint", drawSize=1.0, indices=fix_string)
    
    def mor_test(self):
        print("mor_test callled")
        from sparse_mor_output.reduced_test import Reduced_test
        Reduced_test(self.rootNode)
        
    def place_materials_and_cavities_and_solvers(self):
        """Place solid elastic material and cavities"""
        print("place_materials_and_cavities_and_solvers called")
        # linear solver (with parameters from example file)
        self.rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0',
                                tolerance='0.0000001')






        # Load solvers and animation loop
        # These determine how the simulation moves from one step to the next.
        # the EulerImplicitSolver (I believe) sets up the equations and the
        # SparseLDLSolver is what actually solves them
        # FreeMotionAnimationLoop enables legrangian methods for constraints
        self.rootNode.createObject('FreeMotionAnimationLoop')



        #trail

        self.rootNode.createObject('GenericConstraintSolver', printLog='0', tolerance="1e-15", maxIterations="5000")
        self.rootNode.createObject('DefaultPipeline', name='collisionPipeline', verbose="0")
        self.rootNode.createObject('BruteForceDetection', name="N2")
        self.rootNode.createObject('RuleBasedContactManager', name="Response", response="FrictionContact",
                              rules="0 * FrictionContact?mu=0.5")
        self.rootNode.createObject('DefaultContactManager', response="FrictionContact", responseParams="mu=0.7")
        self.rootNode.createObject('LocalMinDistance', name="Proximity", alarmDistance="2.5", contactDistance="0.5",
                              angleCone="0.01")


        

        # specify solid mechanics
        # self.volume is the elastic material


        self.volume = self.rootNode.createChild('volume')
        self.volume.createObject('EulerImplicitSolver', name='integration')
        self.volume.createObject('SparseLDLSolver', name="solver")

        self.volume.createObject('MeshGmshLoader', name='loader', filename=self.meshFolder + "volume.msh",
                              rotation=self.rotation, translation=self.translation, scale3d=[1.0, 1.0, 1.0])

        self.volume.createObject('TetrahedronSetTopologyContainer', src="@loader", name="container")
        self.volume.createObject('MechanicalObject', template='Vec3d', name='dofs')
        self.volume.createObject('UniformMass', totalMass=self.totalMass, name='mass')
        # if we wanted to do a Neo-Hookean Solid we would do that at this line.
        self.volume.createObject('TetrahedronFEMForceField', template='Vec3d', method='large', name='forcefield',
                                 poissonRatio=self.poissonRatio, youngModulus=self.youngModulus)
        # Constraint solver for corrections, this is how forces between the
        # pressurized cavities are applied to the solid
        #self.volume.createObject('LinearSolverConstraintCorrection', template='Vec3d', solverName='solver')
        self.volume.createObject('GenericConstraintCorrection', solverName='solver')
        # add the cavities
        for ijk in np.ndindex(self.design.shape):
            # if this section is marked as a cavity, create one with the correct index
            if self.design[ijk] == 0:
                i, j, k = ijk

                cavity_i_j_k_str = 'cavity_' + str(i) + "_" + str(j) + "_" + str(k)
                cavity = self.volume.createChild(cavity_i_j_k_str)
                print("\n\n\n\n\n\n\n\n\ncreated", cavity_i_j_k_str)
                # load the specific file, it should be at the correct coordinates and match with a corrisponding
                # one in the volume
                cavity.createObject('MeshSTLLoader', name='loader',
                                 filename=self.meshFolder + cavity_i_j_k_str + '.stl',
                                 translation=self.translation, rotation=self.rotation)
                cavity.createObject('Mesh', src='@loader', name='topo')
                cavity.createObject('MechanicalObject', name='cavity')
                # this pressure constraint is how the cavity is actually controlled
                cavity.createObject('SurfacePressureConstraint',
                                 name="SurfacePressureConstraint",
                                 template='Vec3d', value="6.71401",
                                 triangles='@topo.triangles', drawPressure='0',
                                 drawScale='0.02', valueType="pressure")
                cavity.createObject('BarycentricMapping', name='mapping',
                                 mapForces='false', mapMasses='false')

        if self.with_gui:
            # add visualization for when it is running in a gui.
            # trial

            self.rootNode.createObject('VisualStyle',
                                    displayFlags='showBehaviorModels')
            modelVisu = self.volume.createChild('visu')
            #modelVisu.createObject('MeshSTLLoader', name='loader', filename=self.meshFolder + "volume_collision.stl")

            modelVisu.createObject('OglModel',
                                   src='@../loader',
                                   template='ExtVec3f',
                                   color='0.7 0.7 0.7 0.6')

            modelVisu.createObject('BarycentricMapping')

            robotVisu = self.volume.createChild('robotvisu')
            robotVisu.createObject('TriangleSetTopologyContainer', name='container')
            robotVisu.createObject('TriangleSetTopologyModifier')

            robotVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
            robotVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                                output="@container")
            robotVisu.createObject('OglModel', template='Vec3d', color='0.3 0.2 0.2 0.6', translation=self.translation)
            robotVisu.createObject('IdentityMapping')

            self.rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')

        if self.with_collision:
            
            volume_collision = self.volume.createChild('volume_collision')
            volume_collision.createObject('MeshSTLLoader', name='loader', filename=self.meshFolder+"volume_collision.stl", triangulate="true")
            volume_collision.createObject('TriangleSetTopologyContainer', src='@loader', name='container')
            volume_collision.createObject('MechanicalObject', name='collisiondof', template='Vec3d')
            volume_collision.createObject('TriangleCollisionModel',group="0")
            volume_collision.createObject('LineCollisionModel', group="0")
            volume_collision.createObject('PointCollisionModel',group="0")
            #planeNode.createObject('Triangle', group="1")
            #planeNode.createObject('Line', group="1")
            #planeNode.createObject('Point', group="1")
            volume_collision.createObject('BarycentricMapping')


        if self.with_collision:
            print("BAD CODE")
            # Sub topology this adds "the piece of paper layer"
            self.volume.createObject('BoxROI', name='boxROISubTopo', box='0 0 0 150 -100 1', drawBoxes='true')
            self.volume.createObject('BoxROI', name='membraneROISubTopo', box='0 0 -0.1 150 -100 0.1',
                               computeTetrahedra="false", drawBoxes='true')

            modelSubTopo = self.volume.createChild('modelSubTopo')

            modelSubTopo.createObject('TriangleSetTopologyContainer', position='@../membraneROISubTopo.pointsInROI',
                                      triangles='@../membraneROISubTopo.trianglesInROI', name='container')
            modelSubTopo.createObject('TriangleFEMForceField', template='Vec3d', name='FEM', method='large',
                                    poissonRatio='0.49', youngModulus=float(self.youngModulus)*100*5/7)

    def action(self, act):
        print("action called")
        # For now the action will be a 3d matrix, same dim as disign
        for ijk in np.ndindex(self.design.shape):
            # if this section is marked as a cavity, create one with the correct index
            if self.design[ijk] == 0:
                i, j, k = ijk
                #print('ijk', ijk)

                cavity_i_j_k_str = 'cavity_' + str(i) + "_" + str(j) + "_" + str(k)
                print(cavity_i_j_k_str)
                print("HELLO", self.rootNode.volume.getChildren()[0].getPathName())#.findData(cavity_i_j_k_str).SurfacePressureConstraint.value)) 

                print("HELLO2", self.rootNode.volume.getChild(cavity_i_j_k_str).SurfacePressureConstraint.value)
                self.rootNode.volume.getChild(cavity_i_j_k_str).SurfacePressureConstraint.value = act[ijk]
                #with self.rootNode["volume." + cavity_i_j_k_str + ".SurfacePressureConstraint.value"].writeableArray() as wa:
                #    #print("HERE", wa, act, wa[0], act[ijk])
                #    wa[0] = act[ijk]
                #    del wa
                    
    def observation(self):
        print("observation called")
        return self.rootNode.volume.dofs.getData('position').value
        


def createScene(rootNode):
    '''createScene(rootNode) this is so this works with runSofa'''
    
    print("Example behavior using runSofa animation stepper function")
    

    # this is the Sofa animation function we pass it our animation function
    # and along with the exit function.
    cnt = 0
    ac = list(np.linspace(0,7,71))*10
    ac.sort()
    obs_s = []
    
    def my_animation(target, scn, factor):
        # this animation makes the two cavities oscillate at different
        # frequencies
        action = np.array([[[3*np.sin(factor*np.pi*2),
                               3*np.sin(3*factor*np.pi*2)]]])
        action = np.ones(scn.design.shape)*factor*6
        #action = np.ones(scn.design.shape) * float(factor)
        a = scn.observation()[82]

        
        '''
        if scn.cnt < len(ac):


            obs_s.append(np.array(a))
            action = action - action + ac[scn.cnt]
            scn.cnt += 1

        else:
            print(obs_s)
            np.save('obs.npy', np.array(obs_s))
            action = action - action

        '''
        scn.action(action)


        #print("State: ", np.linalg.norm(a)-2.0, " action: ", action, ' obs ', obs_s[-1] )

    def ExitFunc(target, scn, factor):

        print("Done with animation")
        exit()

    example = "two cell"
    #example = "one cell"
    if example == "two cell":
        # it is 1x1x2 with a cavity in both positions
        design = np.array([[[0, 0]]])
        # this is where the example mesh is stored in the docker file.
        meshpath = '/home/sofauser/workdir/simple_control_policy/mesh/'
        scn = SceneDefinition(rootNode, design=design, meshFolder=meshpath, with_gui=True, constraint="set_z")

    elif example == "one cell":
        design = np.array([[[0]]])
        meshpath = '/home/sofauser/workdir/simple_control_policy/one_cell_robot_mesh/'
        scn = SceneDefinition(rootNode, design=design, meshFolder=meshpath, with_gui=True, collision=False, constraint="set_z")

    elif example == "quadruped":
        design = np.array([[[0, 0, 0, 0, 0]]]) # four legs plus the central cavity
        meshpath = '/home/sofauser/workdir/simple_control_policy/quadruped/'
        scn = SceneDefinition(rootNode, design=design, meshFolder=meshpath, with_gui=True,
                              collision=True, poissonRatio='0.05', totalMass=0.0035, youngModulus='70', constraint="floor")

    # these two lines hook the animation function defined above to the scene graph
    # and makes it so it is run at every time step.
    AnimationManager(rootNode)
    scn.cnt = 0
    scn.design = design
    
    addAnimation(my_animation, {"target": rootNode, "scn":scn}, duration=2.2, mode="once", onDone=ExitFunc)
    #my_animation(rootNode, scn, 11.0)
    return rootNode

