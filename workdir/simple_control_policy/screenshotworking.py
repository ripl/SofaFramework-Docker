# encoding: utf-8
# !/usr/bin/python3

import Sofa
import SofaRuntime
import Sofa.Gui
import numpy as np

class scene_interface:
    """Scene_interface provides step and reset methods"""

    def __init__(self, dt=0.01, max_steps=30):

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

        #confignode = self.root.addChild("Config")
        #confignode.addObject('RequiredPlugin', name="SofaPython3", printLog=False)

        ### create some objects to observe
        self.place_objects_in_scene(self.root)

        # place light and a camera
        self.root.addObject("LightManager")
        self.root.addObject("SpotLight", position=[0,10,0], direction=[0,-1,0])
        self.root.addObject("InteractiveCamera", name="camera", position=[0,10,0],
                            lookAt=[0,0,0], distance=37,
                            fieldOfView=45, zNear=0.63, zFar=55.69)

        # start the simulator
        Sofa.Simulation.init(self.root)
        # start the gui
        Sofa.Gui.GUIManager.Init("Recorded_Episode", "qt")
        print(self.root, __file__)
        Sofa.Gui.GUIManager.createGUI(self.root, __file__)

        # if you want to launch the qt gui use this,
        # Sofa.Gui.GUIManager.MainLoop(self.root)
        # but you can't use
        # Sofa.Simulation.animate in this case


    def place_objects_in_scene(self, root):
        ### these are just some things that stay still and move around
        # so you know the animation is actually happening

        root.gravity = [0, -1, 0]

        # To get your scene to display correctly you will need to set the displayFlags how you want them
        root.addObject("VisualStyle", displayFlags="showAll")

        def Sphere(rootNode, name, position, color):
            # Creating the sphere
            sphere = rootNode.addChild(name)
            sphere.addObject('MechanicalObject', name="mstate", template="Rigid3", position=position)

            #### Visualization of the sphere
            sphereVisu = sphere.addChild("VisualModel")
            sphereVisu.loader = sphereVisu.addObject('MeshObjLoader', name="loader", filename="mesh/ball.obj",
                                                     scale=0.5)
            sphereVisu.addObject('OglModel', name="model", src="@loader", color=color)
            sphereVisu.addObject('RigidMapping')
            return sphere

        Sphere(self.root, "sphere",[-1,0,0,0,0,0,1],[10.0,0.0,0.9])

    def step(self, action):
        # step through time

        # this steps the simulation
        Sofa.Simulation.animate(self.root, self.dt)

        # just to keep track of where we are
        self.current_step += 1


        with self.root.sphere.mstate.position.writeable() as position:
            # set position so the sphere goes around in a circle
            print(position)
            position[0][0] = np.cos(action)*1.0


        # return true if done
        return self.current_step >= self.max_steps


    # save a screenshot from the position of where we set the camera above
    def record_frame(self, filename):
        Sofa.Gui.GUIManager.SaveScreenshot(filename)



def main():
    a = scene_interface()
    done = False
    while not done:
        action = np.pi*2*a.current_step / a.max_steps
        done = a.step(action)
        a.record_frame(str(a.current_step) + ".png")


if __name__ == '__main__':
    main()

