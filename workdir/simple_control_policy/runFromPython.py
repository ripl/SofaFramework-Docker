# encoding: utf-8
#!/usr/bin/python3
import sys
import os

if "SOFA_ROOT" not in os.environ:
        print("WARNING: missing SOFA_ROOT in you environment variable. ") 
        sys.exit(-1)

#sys.path.append(os.path.abspath("./bindings/Sofa/package"))
#sys.path.append(os.path.abspath("./bindings/SofaRuntime/package"))
#sys.path.append(os.path.abspath("./bindings/SofaTypes/package"))

import Sofa
import SofaRuntime
SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT')+"lib/python3/site-packages")

SofaRuntime.PluginRepository.print()

from block_test import createScene
## Register all the common component in the factory. 
SofaRuntime.importPlugin('SofaOpenglVisual')
SofaRuntime.importPlugin("SofaComponentAll")

class MyController(Sofa.Core.Controller):
        def __init__(self, *args, **kwargs):
                Sofa.Core.Controller.__init__(self,*args, **kwargs)
                print("INITED")
                
        def onEvent(self, event):
                print("Event: "+str(event))
               
                
root = Sofa.Core.Node("myroot")
root.addChild("child1")
root.addObject(MyController())
#root = Sofa.Simulation.load("block_test.py")




def simulation_nodes(rootNode):

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject( 'EulerImplicitSolver', name='integration')

    rootNode.addObject( 'SparseLDLSolver', name="solver")

print("\n\n\n\n Got to Screate Scene")
#simulation_nodes(root)
createScene(root)


Sofa.Simulation.init(root)
Sofa.Simulation.print(root)
i = 0.0
while i < 2.0:
        i += 0.001
        print("begin step: "+str(i))
        Sofa.Simulation.animate(root, 0.001)
        print("end step: " +str(i))  
        
