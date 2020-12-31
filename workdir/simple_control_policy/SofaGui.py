import Sofa
import Sofa.Core
import Sofa.Simulation
import Sofa.Gui
import sys
import os
import importlib
import SofaRuntime
import Sofa.Gui

SofaRuntime.importPlugin('SofaOpenglVisual')

import numpy as np



print ("Supported GUIs are " + Sofa.Gui.GUIManager.ListSupportedGUI(","))


root = Sofa.Core.Node("root")
# Create the rest of the scene here...


SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT') + "lib/python3/site-packages")


# Register all the common component in the factory.
SofaRuntime.importPlugin('SofaOpenglVisual')
SofaRuntime.importPlugin("SofaComponentAll")
SofaRuntime.importPlugin('SofaOpenglVisual')
root = Sofa.Core.Node("myroot")

# create the scene
import sceneClass
design= np.array([[[0, 0]]])
dt = 0.001
max_steps=300
meshFolder=os.path.dirname(os.path.abspath(__file__)) + '/mesh/'
#scene = sceneClass.SceneDefinition(root, design=design,
#                          meshFolder=meshFolder, with_gui=True, debug=False)



def Sphere(rootNode, name, position, color):
	#Creating the sphere
	sphere = rootNode.addChild(name)
	sphere.addObject('MechanicalObject', name="mstate", template="Rigid3", position=position)

    	#### Visualization of the sphere
	sphereVisu = sphere.addChild("VisualModel")
	sphereVisu.loader = sphereVisu.addObject('MeshObjLoader', name="loader", filename="mesh/ball.obj", scale=0.5)
	sphereVisu.addObject('OglModel', name="model", src="@loader", color=color)
	sphereVisu.addObject('RigidMapping')
	return sphere

Sphere(root, "hi", "0 0 0", "0 1 0")
# Initialize all components found in the scene
Sofa.Simulation.init(root)


# Launch the GUI
Sofa.Gui.GUIManager.Init("simple_scene", "qt")
Sofa.Gui.GUIManager.createGUI(root, __file__)
Sofa.Gui.GUIManager.MainLoop(root)
#Sofa.Gui.GUIManager.closeGUI()
