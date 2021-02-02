from __future__ import print_function
import os
import sys
import numpy as np
import ctypes
import time
import json
from json import JSONEncoder

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)


class action_decoder(object):
    def __init__(self, read, write):
        self.read = int(read)
        self.write = int(write)
        self.worker = None #LeapWorker()

    def __del__(self):
        #self.worker.stop_render()
        os.close(self.read)
        os.close(self.write)

    def receive(self):
        length = os.read(self.read, 5).decode('utf-8')
        while length == '':
            time.sleep(0.01)
            length = os.read(self.read, 5).decode('utf-8')
        cmd = os.read(self.read, int(length)).decode('utf-8')
        return cmd

    def send(self, msg):
        print("SENDING")
        length = '%05d' % len(msg)
        os.write(self.write, length.encode('utf-8'))
        os.write(self.write, msg.encode('utf-8'))
        print("SENT")

    def recieve_command(self):
        cmd = self.receive()
        if "array" in cmd:
            decodedArrays = json.loads(cmd)
            finalNumpyArray = np.asarray(decodedArrays["array"])
            #print("CMD: decoded", finalNumpyArray)
            return finalNumpyArray
        else:
            return cmd

    def send_observation(self, observation):
        # Serialization
        numpyData = {"array": observation}
        encodedNumpyData = json.dumps(numpyData, cls=NumpyArrayEncoder)
        self.send(encodedNumpyData)

    def run(self):
        try:
            i = 0
            while True:
                cmd = self.receive()
                print("CMD: " + str(cmd))
                decodedArrays = json.loads(cmd)

                finalNumpyArray = np.asarray(decodedArrays["array"])
                print("CMD: decoded", finalNumpyArray)

                if cmd != "close":
                    #self.send(json.dumps(self.worker.get_hands()))
                    numpyArrayOne = np.array([[i, 22, 33], [44, 55, 66], [77, 88, 99]])
                    i += 1
                    # Serialization
                    numpyData = {"array": numpyArrayOne}
                    encodedNumpyData = json.dumps(numpyData, cls=NumpyArrayEncoder)
                    self.send(encodedNumpyData)
                elif cmd == "close":
                    return
                else:
                    raise ValueError("Unkown CMD: %s" % cmd)
        except OSError as e:
            print("ERROR: %s" % str(e))

def main(read, write):
    encoder = action_decoder(read, write)
    encoder.run()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: test.py read write")
    main(sys.argv[1], sys.argv[2])