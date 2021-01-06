# encoding: utf-8
#!/usr/bin/python3

#obs, reward, done, info = self.env.step(action)
#obs = self.env.reset()
import sys
import os
import importlib
import time
import Sofa
import SofaRuntime
import Sofa.Gui
from gym import spaces
from gym.utils import seeding
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
    def __init__(self, env_id, design= np.array([[[0, 0]]]), dt = 0.01, max_steps=300,
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
        self.debug_line_numb = 0
        self.reset_count = 0
        self.msg = ""

        # things requried by DL
        self.env_id = env_id
        self.metadata = None
        self.spec = None
        self.reward_range = (-100000, 100000)
        self.action_space = spaces.Box(low=0.0, high=7.0, shape=design.shape,
                                       dtype=np.float32)
        self.observation_space = spaces.Box(low=-10000.0, high=10000.0,
                                            shape=(3,), dtype=np.float32)

        # start the scene
        obs = self.reset()

        
        
        
    
        
    def debug_output(self, output, filename="/home/sofauser/workdir/debug_output.txt"):
        self.msg = str(self.env_id) + " " + str(self.reset_count) + " " + \
                   str(self.current_step) + " " \
                   + str(self.debug_line_numb) \
                   + " " + output + "\n"
        file1 = open(filename, "a")  # append mode
        file1.write(self.msg)
        file1.close()
        self.debug_line_numb += 1
        
    def seed(self, seed=None):
        
        """Seed the PRNG of this space. """
        self._np_random, seed = seeding.np_random(seed)
        return [seed]
        
    def reset(self):
        """Reset the simulation, return the initial observation"""
        self.root = None
        self.scene = None
        self.sim_time = 0.0
        self.current_step = 0
        self.debug_line_numb = 0
        self.reset_count += 1
        self.debug_line_numb = 0
        self.msg = ""
        
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
        

        return self.get_observation()
    
    def close(self):
        pass
        
    def reward(self, observation):
        """Returns reward given an observation"""
        # for the time being this is just measuring if the robot can stretch to
        # 1.25 of its original length with dense reward.
        '''
        max_val = np.max(observation)
        min_val = np.min(observation)
        
        self.debug_output(" max val "+str(max_val)
                              +" min val "+str(min_val))
        
        rwd = -np.power( np.abs(max_val-min_val)-0.25, 2)+0.0025

        #if rwd < self.reward_range[0]:
        #    rwd = self.reward_range[0]
        '''
        #simplified reward is just the radius of the baloon", sub 2 to take away starting value, 0.25 to set desired point and add a little so it starts at zero again
        rwd = -1.0*np.power(np.linalg.norm(observation)-2.0000000000000004 - 0.20, 2) + np.power(0.20,2)
        rwd = rwd / 0.04
        self.debug_output("observation: "+repr(observation) +"\nreward: "+repr(rwd))

        return rwd
        
    def get_observation(self):
        
        observation = self.scene.observation()
        observation = observation[82] 
        observation = observation.flatten()
        #print(observation)
        #observation = self.scene.observation().flatten()
        if np.isnan(np.sum(observation)):
            self.debug_output("found nan in observation "+str(time.time()),
                              filename="/home/sofauser/workdir/debug_nan.txt")
        np.clip(observation, self.observation_space.low,
                self.observation_space.high, out=observation)
        
        return observation
    
    def step(self, action):
        "applies action to the scene and returns observation, reward, done,info"
        # action: 3d array same shape as design. 
        # For now: the action array is the pressure in each cavity at each position
        # in the lattice. If there is no corresponding cavity to a non-zero value
        # nothing is done with that non-zero value.
        
        if np.isnan(np.sum(action)):
            self.debug_output("found nan in action "+str(time.time()),
                              filename="/home/sofauser/workdir/debug_nan.txt")
            
        np.clip(action, self.action_space.low, self.action_space.high, out=action)
        
        self.debug_output("action: " + repr(action))
        


        # step the clock
        self.current_step += 1
        self.sim_time += self.dt 
        done = self.max_steps <= self.current_step

        # apply the action
        self.scene.action(action)

        # step the simulator
        Sofa.Simulation.animate(self.root, self.dt)
        
        # get the objservation and calculate the reward
        obs = self.get_observation()
        
        rwrd = self.reward(obs)

        return obs, rwrd, done, {}

        
        

def main():
    meshpath = '/home/sofauser/workdir/simple_control_policy/one_cell_robot_mesh/'

    # it is 1x1x2 with a cavity in both positions
    #design = np.array([[[0, 0]]])
    design = np.array([[[ 0]]])

    a = scene_interface(0, design=design, dt = 0.001, max_steps=300,
                 meshFolder=meshpath)
    
    done = False
    
    total = 0
    while not done:
        factor = a.current_step / a.max_steps
        action = 5*np.cos(factor*np.pi*2)*np.ones(design.shape)
        action = action*0 + 6.5
        obs2, reward, done, info  = a.step(action)
        total += reward
        print('reward:', reward, " total reward ", total)
        print('action:', action)
   
 

if __name__ == '__main__':
    main()

