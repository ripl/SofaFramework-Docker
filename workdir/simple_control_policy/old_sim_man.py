# encoding: utf-8
#!/usr/bin/python3

#obs, reward, done, info = self.env.step(action)
#obs = self.env.reset()
def entire(name):
    print(name)
    import sys
    import os

    import Sofa
    import SofaRuntime

    from old_scene import createScene

    SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT')+"lib/python3/site-packages")
    SofaRuntime.PluginRepository.print()
    ##�Register all the common component in the factory. 
    SofaRuntime.importPlugin('SofaOpenglVisual')
    SofaRuntime.importPlugin("SofaComponentAll")

    class MyController(Sofa.Core.Controller):
        def __init__(self, *args, **kwargs):
                Sofa.Core.Controller.__init__(self,*args, **kwargs)
                print("INITED")
                
        def onEvent(self, event):
                print("Event: "+str(event))
               
                
    def main():            
        root = Sofa.Core.Node("myroot")
        root.addObject(MyController())
        createScene(root)
        Sofa.Simulation.init(root)
        Sofa.Simulation.print(root)
        i = 0.0
        while i < 0.25:
            i += 0.01
            #if i == 0.25:
            #    root.reset()
            
            print("begin step: "+str(i))
            Sofa.Simulation.animate(root, 0.01)
            print("end step: " +str(i))  
   

    main()

