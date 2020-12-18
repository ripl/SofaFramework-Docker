# encoding: utf-8
#!/usr/bin/python3

#obs, reward, done, info = self.env.step(action)
#obs = self.env.reset()
import sys
import os
import importlib
import Sofa
import SofaRuntime
import Sofa.Gui


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
                 meshFolder=os.path.dirname(os.path.abspath(__file__)) + '/mesh/',
                 debug=False):

        # it is 1x1x2 with a cavity in both positions
        
        # For now: Design is a 3 dim array that specifies if it is cavity or material
        # at each discrete cube in the lattice that makes up the robot
        self.design = design

        # dt (seconds) the time step of the simulation, default 0.001
        self.dt = dt

        # max_steps, how long the simulator should run. Total length: dt*max_steps
        self.max_steps = max_steps

        #meshFolder, where the meshes are stored. The solid mesh is always
        # volume.msh, and each cavity if it exists is called cavity_{i}_{j}_{k}.stl
        # where that would be a format string and i,j,k are the idicies in the latus
        self.meshFolder = meshFolder
        
        # root node in the simulator
        self.root = None
        # the scene object
        self.scene = None
        # to keep track of sim time in seconds
        self.sim_time = 0.0
        # the current step in the simulation
        self.current_step = 0
        
        self.debug = debug
        self.reset()
    
    def reset(self):
        """Reset the simulation, return the initial observation"""
        self.root = None
        self.scene = None
        self.sim_time = 0.0
        self.current_step = 0
        
        #importlib.reload(os)
        #importlib.reload(Sofa)
        #importlib.reload(SofaRuntime)
        #importlib.reload(Sofa.Gui)
        #importlib.reload(sceneClass)
        
        # every time we reset we setup the simulator fresh.
        SofaRuntime.PluginRepository.addFirstPath(os.getenv('SOFA_ROOT') + "lib/python3/site-packages")
        
        
        # If needed this will print the scene graph
        if self.debug:
            SofaRuntime.PluginRepository.print()
        
        # Register all the common component in the factory.
        SofaRuntime.importPlugin('SofaOpenglVisual')
        SofaRuntime.importPlugin("SofaComponentAll")
        self.root = Sofa.Core.Node("myroot")
        
        # create the scene
        self.scene = sceneClass.SceneDefinition(self.root, design=self.design,
                          meshFolder=self.meshFolder, with_gui=True, debug=self.debug)
    

        # This is what starts the environment up (I think) and calling this is 
        # what clears out the old simulation. (Not entirely sure, if this doesn't
        # work, try uncommenting the importlib commands above
        Sofa.Simulation.init(self.root)
        if self.debug:
            Sofa.Simulation.print(self.root)
        

        return self.scene.observation()
        
    def reward(self, observation):
        """Returns reward given an observation"""
        # for the time being this is just measuring if the robot can stretch to
        # 1.25 of its original length with dense reward.

        max_val = np.max(observation[:,2])
        min_val = np.min(observation[:,2])
        if self.debug:
            print(max_val, min_val, " max val min val")
        
        return -np.power( np.abs(max_val-min_val)-0.25, 2)
        
    def step(self, action):
        "applies action to the scene and returns observation, reward, done,info"
        # action: 3d array same shape as design. 
        # For now: the action array is the pressure in each cavity at each position
        # in the lattice. If there is no corresponding cavity to a non-zero value
        # nothing is done with that non-zero value.


        # step the clock
        self.current_step += 1
        self.sim_time += self.dt 
        done = self.max_steps < self.current_step

        # apply the action
        self.scene.action(action)

        # step the simulator
        Sofa.Simulation.animate(self.root, self.dt)
        
        # get the objservation and calculate the reward
        obs = self.scene.observation()
        rwrd = self.reward(obs)

        return obs, rwrd, done, None

        
        

def main():
    a = scene_interface(design= np.array([[[0, 0]]]), dt = 0.001, max_steps=300,
                 meshFolder=os.path.dirname(os.path.abspath(__file__)) + '/mesh/')
    
    done = False
    
    while not done:
        factor = a.current_step / a.max_steps
        action = np.array([[[3*np.sin(factor*np.pi*2),
                                   3*np.sin(3*factor*np.pi*2)]]])
        obs, reward, done, info  = a.step(action)
        
        print('reward:', reward)

    a.reset()
    
    done = False

    while not done:
        factor = a.current_step / a.max_steps
        action = np.array([[[3*np.sin(factor*np.pi*2),
                                   3*np.sin(3*factor*np.pi*2)]]])
        obs2, reward, done, info  = a.step(action)

        print('reward:', reward)

    print(np.sum(obs2-obs))
 
if __name__ == '__main__':
    main()
