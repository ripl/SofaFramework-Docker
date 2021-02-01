#/usr/bin/python3
import sys, os
dir_path = os.path.dirname(os.path.realpath(__file__))
#sys.path.insert(0, os.path.join(dir_path, 'lib'))
#sys.path.insert(0, os.path.join(dir_path, 'lib/x64'))
import subprocess
import json
import time
from json import JSONEncoder
import numpy as np

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)




class ActionEncoder(object):
    def __init__(self, runSofa_cmd_args, cwd_dir, additional_python_args=None):
        self.r_send, self.w_send = os.pipe()
        self.r_recv, self.w_recv = os.pipe()
        os.set_inheritable(self.r_send, True)
        os.set_inheritable(self.w_recv, True)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        #self.daemon = subprocess.Popen(['leapd'])
        #self.encoder = subprocess.Popen([
        #        'python2', os.path.join(dir_path, 'command_decoder.py'),
        #        f'{self.r_send}', f'{self.w_recv}'
        #    ], close_fds=False)
        self.encoder = subprocess.Popen(runSofa_cmd_args + "--argv " + additional_python_args +
            f' --r_send {self.r_send} '+ f' --w_recv {self.w_recv}', cwd=cwd_dir, shell=True, close_fds=False)
        print('waiting for deamon to start...')
        delay = 1
        for i in range(delay):
            print(f"Sleeping for {delay - i} seconds...")
            time.sleep(1)

    def __del__(self):
        self.send('close')
        os.close(self.w_send)
        os.close(self.r_recv)
        #self.daemon.terminate()

    def send(self, msg):
        length = f'{len(msg):05}'
        os.write(self.w_send, length.encode('utf-8'))
        os.write(self.w_send, msg.encode('utf-8'))

    def receive(self):
        length = os.read(self.r_recv, 5).decode('utf-8')
        msg = os.read(self.r_recv, int(length)).decode('utf-8')
        return json.loads(msg)




class SimulationInterface(object):
    def __init__(self, runSofa_cmd_args, cwd_dir, additional_python_args=None):
        self.interface = ActionEncoder(runSofa_cmd_args, cwd_dir=cwd_dir, additional_python_args=additional_python_args)

    def step(self, action):
        # must make sure both action and obs are np arrays
        action = {"array" : action}
        encoded_action = json.dumps(action, cls=NumpyArrayEncoder)
        self.interface.send(encoded_action)
        obs = self.interface.receive()
        return obs



if __name__ == '__main__':
    leap = SimulationInterface()

    for i in range(10):
        t = time.time()
        print(leap.step(np.array([1,2,i])))
        print(time.time() - t)
        time.sleep(1)