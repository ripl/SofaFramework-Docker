import Sofa
import Sofa.Core
import Sofa.Simulation
import Sofa.Gui

import SofaRuntime
SofaRuntime.importPlugin('SofaOpenglVisual')

print ("Supported GUIs are " + Sofa.Gui.GUIManager.ListSupportedGUI(","))

root = Sofa.Core.Node("root")
# Create the rest of the scene here...

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


Sphere(root, "hi", "0 0 1", "0 0.3 1 1")

# Initialize all components found in the scene
Sofa.Simulation.init(root)

# Launch the GUI
Sofa.Gui.GUIManager.Init("simple_scene", "qt")
Sofa.Gui.GUIManager.createGUI(root, __file__)

Sofa.Gui.GUIManager.SaveScreenshot('test.png')
print(Sofa.Gui.GUIManager.SaveScreenshot.__doc__)
#Sofa.Gui.GUIManager.MainLoop(root)


Sofa.Gui.GUIManager.closeGUI()
