import Sofa
import sys
import os
from stlib.physics.constraints import FixedBox
from stlib.scene import Scene
from splib.animation import animate
from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity

import numpy as np

path = os.path.dirname(os.path.abspath(__file__)) + '/'
meshpath = path


def createSceneReal(rootNode, dt):
    length_scale = "0.500"
    disk_msh = 'disk_'+length_scale+'.msh'
    disk_inside_stl = 'disk_inside'+length_scale+'.stl'
    
    

    
    
    
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 9.8], dt=dt)
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')


    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    YoungModulus = 1800
    InitialValue = 1000.0
    Translation="0 0 0"
    Bunny = ElasticMaterialObject(name="disk",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+disk_msh,
                                  surfaceMeshFileName=meshpath+disk_inside_stl,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation="0 0 0")
    
    fixed_const_str = ""
    fixed_const_lst = [274, 309, 344, 345, 770, 783, 807] 
    for i in fixed_const_lst:
        fixed_const_str = fixed_const_str + " " + str(i)
    
    print "Fixed stuff", fixed_const_lst, fixed_const_str

    Bunny.createObject("FixedConstraint", indices=fixed_const_str)
    '''
    cavity = PneumaticCavity(name='cavity', attachedAsAChildOf=Bunny,
                             surfaceMeshFileName=meshpath + disk_inside_stl, valueType='pressure',
                             initialValue=0.0, translation=Translation)
    '''
    cavity = Bunny.createChild('cavity')
    cavity.createObject('MeshSTLLoader', name='loader', filename=meshpath + disk_inside_stl,
                        translation="0 0 0")
    cavity.createObject('Mesh', src='@loader', name='topo')
    cavity.createObject('MechanicalObject', name='cavity')
    cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0.0001",
                        triangles='@topo.triangles', visualization='0', showVisuScale='0.0002', valueType="pressure")
    cavity.createObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')

    BunnyVisu = Bunny.createChild('visu')
    BunnyVisu.createObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.createObject('TriangleSetTopologyModifier')
    BunnyVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    BunnyVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    BunnyVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    BunnyVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    BunnyVisu.createObject('IdentityMapping')
    
    return Bunny



def createScene(rootNode):
    dt = 0.001
    length_scale = '0.500'
    disk_msh = 'disk_' + length_scale + '.msh'



    

    

    def animation(target, factor):

        pressureValue = target.disk.cavity.SurfacePressureConstraint.findData('value').value[0][0] + 0.01
        pressureValue = np.abs(np.sin(factor)*0.03)
        if pressureValue > 0.025:
            pressureValue = 0.02
        target.disk.cavity.SurfacePressureConstraint.findData('value').value = str(pressureValue)
        #This is a dummy animation function
        #I would like to use Model Order Reducation on a model that I 
        #animate, but I want to figure out how to do it with the default one first
        #print "Factor ", factor, " ", pressureValue

    

    createSceneReal(rootNode, dt)
    animate(animation, {"target": rootNode}, duration=2, mode="once")

    return rootNode
    

    
