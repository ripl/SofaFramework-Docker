# encoding: utf-8
# !/usr/bin/python3

import Sofa
import SofaRuntime
import Sofa.Gui


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


        # Register all the common component in the factory.
        SofaRuntime.importPlugin('SofaOpenglVisual')
        SofaRuntime.importPlugin("SofaComponentAll")

        self.root = Sofa.Core.Node("myroot")


        ### create some objects to observe
        self.place_objects_in_scene(self.root)

        # place light and a camera
        self.root.addObject("LightManager")
        self.root.addObject("SpotLight", position=[0,10,0], direction=[0,-1,0])
        self.root.addObject("InteractiveCamera", name="camera", position=[0,10, 0],
                            lookAt=[0,0,0], distance=37,
                            fieldOfView=45, zNear=0.63, zFar=55.69)

        # start the simulator
        Sofa.Simulation.init(self.root)
        # start the gui
        Sofa.Gui.GUIManager.Init("Recorded_Episode", "qt")
        Sofa.Gui.GUIManager.createGUI(self.root, __file__)


    def place_objects_in_scene(self, root):
        ### these are just some things that stay still and move around
        # so you know the animation is actually happening
        root.gravity = [0, -1., 0]
        root.addObject("VisualStyle", displayFlags="showWireframe showBehaviorModels showAll")
        root.addObject("MeshGmshLoader", name="meshLoaderCoarse",
                       filename="mesh/liver.msh")
        root.addObject("MeshObjLoader", name="meshLoaderFine",
                       filename="mesh/liver-smooth.obj")

        root.addObject("EulerImplicitSolver")
        root.addObject("CGLinearSolver", iterations="200",
                        tolerance="1e-09", threshold="1e-09")


        liver = root.addChild("liver")

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




    def step(self):
        # step through time

        # this steps the simulation
        Sofa.Simulation.animate(self.root, self.dt)

        # just to keep track of where we are
        self.current_step += 1

        ### A better example would also show how to read and edit values through scripts
        # which would likely be useful if you are running without a normal gui

        # return true if done
        return self.current_step >= self.max_steps


    # save a screenshot from the position of where we set the camera above
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

