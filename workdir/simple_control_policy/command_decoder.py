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


class LeapEncoder(object):
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
                elif cmd == "render":
                    #self.worker.render()
                    print("COMMAND was rdner")
                elif cmd == "close":
                    return
                else:
                    raise ValueError("Unkown CMD: %s" % cmd)
        except OSError as e:
            print("ERROR: %s" % str(e))

'''
class ImageListener(Leap.Listener):

    def on_images(self, controller):
        for i, im in enumerate(controller.images):
            ctype_array_def = ctypes.c_ubyte * im.width * im.height
            # as ctypes array
            as_ctype_array = ctype_array_def.from_address(int(im.data_pointer))
            # as numpy array
            im = np.ctypeslib.as_array(as_ctype_array)
            cv2.imshow('im%d' % i, im)
        cv2.waitKey(1)


class LeapWorker(object):
    def __init__(self):
        self.controller = Leap.Controller()
        self.controller.set_policy(Leap.Controller.POLICY_BACKGROUND_FRAMES)
        self.image_listener = None

    def get_hands(self):
        frame = self.controller.frame()
        hands = []
        for hand in frame.hands:
            hands.append({
                'is_left': hand.is_left,
                'palm': [
                    hand.palm_position.x,
                    hand.palm_position.y,
                    hand.palm_position.z,
                ],
                'fingers': {
                    finger.type: [
                        [
                            finger.joint_position(Finger.JOINT_MCP).x,
                            finger.joint_position(Finger.JOINT_MCP).y,
                            finger.joint_position(Finger.JOINT_MCP).z
                        ],
                        [
                            finger.joint_position(Finger.JOINT_PIP).x,
                            finger.joint_position(Finger.JOINT_PIP).y,
                            finger.joint_position(Finger.JOINT_PIP).z
                        ],
                        [
                            finger.joint_position(Finger.JOINT_DIP).x,
                            finger.joint_position(Finger.JOINT_DIP).y,
                            finger.joint_position(Finger.JOINT_DIP).z
                        ],
                        [
                            finger.joint_position(Finger.JOINT_TIP).x,
                            finger.joint_position(Finger.JOINT_TIP).y,
                            finger.joint_position(Finger.JOINT_TIP).z
                        ]
                    ]
                    for finger in hand.fingers
                }
            })
        return hands

    def render(self):
        if self.image_listener is None:
            print("HIII")
            self.controller.set_policy(Leap.Controller.POLICY_IMAGES)
            self.image_listener = ImageListener()
            self.controller.add_listener(self.image_listener)

    def stop_render(self):
        if self.image_listener is not None:
            self.controller.remove_listener(self.image_listener)
            self.controller.clear_policy(Leap.Controller.POLICY_IMAGES)
            self.image_listener = None
            cv2.destroyAllWindows()

'''
def main(read, write):
    encoder = LeapEncoder(read, write)
    encoder.run()


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: test.py read write")
    main(sys.argv[1], sys.argv[2])