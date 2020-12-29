# encoding: utf-8
#!/usr/bin/python3

import os
from stlib3.scene import Scene
import numpy as np
import timeit
from splib.animation import AnimationManager, addAnimation


class SceneDefinition:
    """This class handles the scene creation of the robot model"""
    def __init__(self, rootNode, design=np.zeros([1,1,1]), meshFolder="",
                 poissonRatio=0.3, youngModulus= 18000, totalMass = 1.0,
                 dt=0.001, with_gui = False, debug=False):
        """initialize the class"""
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

        # this is the position and orientation of the robot. It is not fully
        # implemented so I think it is bedst to leave it as is.
        self.rotation = [0.0, 0.0, 0.0]
        self.translation = [0.0, 0.0, 0.0]
        
        # this will be the base of the robot in the scene graph
        self.volume = None

        # TODO: check if Scene() is actually needed (I don't think it is)
        Scene(self.rootNode, gravity=[0, 0.0, 0], dt=dt)

        required_plugins = ['SofaTopologyMapping', 'SofaOpenglVisual', 'SofaSparseSolver', 'SofaConstraint',
                            'SofaGeneralLoader', 'SoftRobots']
        for i in required_plugins:
            self.rootNode.addObject("RequiredPlugin", name='req_p' + i, pluginName=i)

        # create all the material and cavities
        self.place_materials_and_cavities_and_solvers()
        
        # fix the base (all nodes where z=0)
        self.fix_z()

    def fix_z(self):
        '''Fix all vertexes in mesh where z=0, this is so that one the robot doesn't
        fly off to infinity if it gets pushed a little bit, and two so it has something
        to push against while for now: it does simple shape rewards'''
        vertexes = self.rootNode.volume.loader.position.array()
        fix_string = " "
        for i, pos in enumerate(vertexes):
            if pos[2] == 0.0:
                #print(i,pos)
                fix_string = fix_string + " " + str(i)
        
        # this adds a constraint that fixes these vertexes
        self.volume.addObject("FixedConstraint", indices=fix_string)
    
    
    def place_materials_and_cavities_and_solvers(self):
        """Place solid elastic material and cavities"""
        # linear solver (with parameters from example file)
        self.rootNode.addObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0',
                                tolerance='0.0000001')
        
        # self.volume is the elastic material
        self.volume = self.rootNode.addChild('volume')
        self.volume.addObject('MeshGmshLoader', name='loader', filename=self.meshFolder+"volume.msh",
                            rotation=self.rotation, translation=self.translation, scale3d=[1.0,1.0,1.0])
        
        
        # Load solvers and animation loop
        # These determine how the simulation moves from one step to the next.
        # the EulerImplicitSolver (I believe) sets up the equations and the
        # SparseLDLSolver is what actually solves them
        # FreeMotionAnimationLoop enables legrangian methods for constraints
        self.rootNode.addObject('FreeMotionAnimationLoop')
        self.rootNode.addObject( 'EulerImplicitSolver', name='integration')
        self.rootNode.addObject( 'SparseLDLSolver', name="solver")
        
        
        # specify solid mechanics
        self.volume.addObject('TetrahedronSetTopologyContainer', src="@loader", name="container")
        self.volume.addObject('MechanicalObject', template='Vec3d', name='dofs')
        self.volume.addObject('UniformMass', totalMass=self.totalMass, name='mass')
        # if we wanted to do a Neo-Hookean Solid we would do that at this line.
        self.volume.addObject('TetrahedronFEMForceField', template='Vec3d', method='large', name='forcefield',
                                 poissonRatio=self.poissonRatio, youngModulus=self.youngModulus)
        # Constraint solver for corrections, this is how forces between the
        # pressurized cavities are applied to the solid
        self.volume.addObject('LinearSolverConstraintCorrection', template='Vec3d', solverName='../solver')
        
        # add the cavities
        for ijk in np.ndindex(self.design.shape):
            # if this section is marked as a cavity, create one with the correct index
            if self.design[ijk] == 0:
                i, j, k = ijk
                cavity = self.volume.addChild(f'cavity_{i}_{j}_{k}')
                
                # load the specific file, it should be at the correct coordinates and match with a corrisponding
                # one in the volume
                cavity.addObject('MeshSTLLoader', name='loader',
                                 filename=self.meshFolder + f'cavity_{i}_{j}_{k}.stl',
                                 translation=self.translation, rotation=self.rotation)
                cavity.addObject('Mesh', src='@loader', name='topo')
                cavity.addObject('MechanicalObject', name='cavity')
                # this pressure constraint is how the cavity is actually controlled
                cavity.addObject('SurfacePressureConstraint',
                                 name="SurfacePressureConstraint",
                                 template='Vec3d', value="0.0001",
                                 triangles='@topo.triangles', drawPressure='0',
                                 drawScale='0.0002', valueType="pressure")
                cavity.addObject('BarycentricMapping', name='mapping',
                                 mapForces='false', mapMasses='false')

        if self.with_gui:
            # add visualization for when it is running in a gui.
            self.rootNode.addObject('VisualStyle',
                                    displayFlags='showVisualModels hideBehaviorModels showCollisionModels ' +
                                                 'hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

            robotVisu = self.volume.addChild('robotvisu')
            robotVisu.addObject('TriangleSetTopologyContainer', name='container')
            robotVisu.addObject('TriangleSetTopologyModifier')
    
            robotVisu.addObject('TriangleSetGeometryAlgorithms', template='Vec3d')
            robotVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                                output="@container")
            robotVisu.addObject('OglModel', template='Vec3d', color='0.3 0.2 0.2 0.6', translation=self.translation)
            robotVisu.addObject('IdentityMapping')
            self.rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')


    
    def action(self, act):
        # For now the action will be a 3d matrix, same dim as disign
        for ijk in np.ndindex(self.design.shape):
            # if this section is marked as a cavity, create one with the correct index
            if self.design[ijk] == 0:
                i, j, k = ijk
                
                with self.rootNode[f"volume.cavity_{i}_{j}_{k}.SurfacePressureConstraint.value"].writeableArray() as wa:
                    wa[0] = act[ijk]
                    
    def observation(self):
        return self.rootNode.volume.dofs.position.value
        


def createScene(rootNode):
    '''createScene(rootNode) this is so this works with runSofa'''
    
    print("Example behavior using runSofa animation stepper function")
    

    # this is the Sofa animation function we pass it our animation function
    # and along with the exit function.
    def my_animation(target, scn, factor):
        # this animation makes the two cavities oscillate at different
        # frequencies
        if factor > 0.04:
            scn.action(np.array([[[3*np.sin(factor*np.pi*2),
                                   3*np.sin(3*factor*np.pi*2)]]]))
        
        print(factor)
        #print("State: ", scn.observation())
        
    def ExitFunc(target, scn, factor):
        
        print("Done with animation")
        exit()
    
    # this is where the example mesh is stored in the docker file.
    meshpath = '/home/sofauser/workdir/simple_control_policy/mesh/'
    
    # it is 1x1x2 with a cavity in both positions
    design = np.array([[[0, 0]]])
    scn = SceneDefinition(rootNode, design=design, meshFolder=meshpath, with_gui=True)
    
    # these two lines hook the animation function defined above to the scene graph
    # and makes it so it is run at every time step.
    AnimationManager(rootNode)
    addAnimation(my_animation, {"target": rootNode, "scn":scn}, duration=2.2, mode="once", onDone=ExitFunc)

    return rootNode
