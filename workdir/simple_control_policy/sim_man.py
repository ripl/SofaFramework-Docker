# encoding: utf-8
#!/usr/bin/python3

#obs, reward, done, info = self.env.step(action)
#obs = self.env.reset()
import sys
import os
import importlib
import Sofa
import SofaRuntime
import numpy as np
import sceneClass


#class MyController(Sofa.Core.Controller):
#    def __init__(self, *args, **kwargs):
#            Sofa.Core.Controller.__init__(self,*args, **kwargs)
#            print("INITED")
#
#    def onEvent(self, event):
#            print("Event: "+str(event))
# root.addObject(MyController())


class scene_interface:
    """Scene_interface provides step and reset methods"""
    def __init__(self, design= np.array([[[0, 0]]]), dt = 0.001, max_steps=300,
                 meshFolder=os.path.dirname(os.path.abspath(__file__)) + '/mesh/'):
        self.design = design
        self.dt = dt
        self.max_steps = max_steps
        self.meshFolder = meshFolder
        self.root = None
        self.scene = None
        self.sim_time = 0.0
        self.current_step = 0
        self.reset()
    
    def reset(self):
        self.root = None
        self.scene = None
        self.sim_time = 0.0
        self.current_step = 0
        
        importlib.reload(os)
        importlib.reload(Sofa)
        importlib.reload(SofaRuntime)
        importlib.reload(sceneClass)
        SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT') + "lib/python3/site-packages")
        SofaRuntime.PluginRepository.print()
        # Register all the common component in the factory.
        SofaRuntime.importPlugin('SofaOpenglVisual')
        SofaRuntime.importPlugin("SofaComponentAll")
    
        self.root = Sofa.Core.Node("myroot")
    


        # it is 1x1x2 with a cavity in both positions
        self.scene = sceneClass.SceneDefinition(self.root, design=self.design,
                          meshFolder=self.meshFolder, with_gui=True)
    


        Sofa.Simulation.init(self.root)
        Sofa.Simulation.print(self.root)
        
        return self.scene.observation()
        
    def reward(self, observation):
        max_val = np.max(observation[:,:,2])
        min_val = np.min(observation[:,:,2])
        return -np.power( np.abs(max_val-min_val)-2.5, 2)
        
    def step(self, action):
        "applies action to the scene and returns observation, reward, done,info"
        self.current_step += 1
        self.scene.action(action)
        Sofa.Simulation.animate(self.root, self.dt)
        obs = self.scene.observation()
        rwrd = self.reward(obs)
        done = self.max_steps < self.current_step
        return obs, rwrd, done, None

        
        

def main():
    a = scene_interface(design= np.array([[[0, 0]]]), dt = 0.001, max_steps=300,
                 meshFolder=os.path.dirname(os.path.abspath(__file__)) + '/mesh/')
    
    done = False
    
    while not done:
        factor = a.current_step / a.max_steps
        action = np.array([[[3*np.sin(factor*np.pi*2),
                                   3*np.sin(3*factor*np.pi*2)]]])
        _, reward, done, _  = a.step(action)
        
        print(reward)