# encoding: utf-8
# !/usr/bin/python3

import os
import Sofa
import SofaRuntime
import Sofa.Gui
import numpy as np


SofaRuntime.importPlugin('SofaOpenglVisual')





class scene_interface:
    """Scene_interface provides step and reset methods"""

    def __init__(self, dt=0.01, max_steps=300):

        self.dt = dt

        # max_steps, how long the simulator should run. Total length: dt*max_steps
        self.max_steps = max_steps

        # root node in the simulator
        self.root = None
        # the current step in the simulation
        self.current_step = 0

        # every time we reset we setup the simulator fresh.
        SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT') + "lib/python3/site-packages")

        # Register all the common component in the factory.
        SofaRuntime.importPlugin('SofaOpenglVisual')
        SofaRuntime.importPlugin("SofaComponentAll")

        self.root = Sofa.Core.Node("myroot")

        self.place_objects_in_scene(self.root)
        ### create some objects to observe

        self.root.addObject("LightManager")
        self.root.addObject("SpotLight", position=[0,10,0], direction=[0,-1,0])
        self.root.addObject("InteractiveCamera", name="camera", position=[0,10, 0],
                            lookAt=[0,0,0], distance=37,
                            fieldOfView=45, zNear=0.63, zFar=55.69)

        Sofa.Simulation.init(self.root)

        Sofa.Gui.GUIManager.Init("Recorded_Episode", "qt")
        Sofa.Gui.GUIManager.createGUI(self.root, __file__)


    def place_objects_in_scene(self, root):
        #root.addObject("MechanicalObject", name="dofs", template="Rigid3",
        #               position=[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]],
        #               showObject=True, showObjectScale=1)
        #
        #root.addObject("MeshObjLoader", name="loader", filename="mesh/Armadillo_simplified.obj", scale3d="0.1 0.1 0.1")
        # root.addObject("OglModel", name="visual", src="@loader", color="1 0 0 0.2")
        def Sphere(rootNode, name, position, color):
            # Creating the sphere
            sphere = rootNode.addChild(name)
            sphere.addObject('MechanicalObject', name="dofs", template="Rigid3", position=position)

            #### Visualization of the sphere
            sphereVisu = sphere.addChild("VisualModel")
            sphereVisu.loader = sphereVisu.addObject('MeshObjLoader', name="loader", filename="mesh/ball.obj",
                                                     scale=0.5)
            sphereVisu.addObject('OglModel', name="model", src="@loader", color=color)
            sphereVisu.addObject('RigidMapping')
            return sphere

        Sphere(root, "sphere", "0 0 1", "0 0.3 1 1")

        def liver(root):
            root.gravity = [0, -1., 0]
            root.addObject("VisualStyle", displayFlags="showWireframe showBehaviorModels")
            root.addObject("MeshGmshLoader", name="meshLoaderCoarse",
                           filename="mesh/liver.msh")
            root.addObject("MeshObjLoader", name="meshLoaderFine",
                           filename="mesh/liver-smooth.obj")

            liver = root.addChild("liver")
            liver.addObject("EulerImplicitSolver")
            liver.addObject("CGLinearSolver", iterations="200",
                            tolerance="1e-09", threshold="1e-09")
            liver.addObject("TetrahedronSetTopologyContainer",
                            name="topo", src="@../meshLoaderCoarse" )
            liver.addObject("TetrahedronSetGeometryAlgorithms",
                            template="Vec3d", name="GeomAlgo")
            liver.addObject("MechanicalObject",
                            template="Vec3d",
                            name="MechanicalModel", showObject="1", showObjectScale="3")

            liver.addObject("TetrahedronFEMForceField", name="fem", youngModulus="1000",
                            poissonRatio="0.4", method="large")

            liver.addObject("MeshMatrixMass", massDensity="1")
            liver.addObject("FixedConstraint", indices="2 3 50")
            liver.addObject("OglModel", name="VisualModel", src="@../meshLoaderCoarse")
            #visual = liver.addChild("visual")
            #visual.addObject("VisualModel", src="@../../meshLoaderFine")
            #visual.addObject("BarycentricMapping",
            #                name="VMapping" ,
            #                 input="@../MechanicalModel",
            #                 output="@VisualModel")

        liver(root)



    def step(self):
        Sofa.Simulation.animate(self.root, self.dt)
        self.current_step += 1
        factor = np.sin(self.current_step / self.max_steps)
        a = self.root.liver
        #print(dir(a))
        #print(a)
        # move the thing we are looking at for something to watch
        #with a.writeableArray() as wa:
        #    wa[:] = np.array([0.0, 0.0, self.current_step, self.current_step, 0, 0])


        return self.current_step >= self.max_steps



    def record_frame(self, filename):
        Sofa.Gui.GUIManager.SaveScreenshot(filename)


def main():



    a = scene_interface()

    done = False
    while not done:
        factor = a.current_step
        done = a.step()
        a.record_frame(str(factor) + ".png")


if __name__ == '__main__':
    main()

