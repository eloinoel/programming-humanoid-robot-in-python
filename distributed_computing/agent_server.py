'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import pickle
from os import listdir
from joint_control.keyframes import *
from time import sleep
import numpy as np

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from kinematics.inverse_kinematics import InverseKinematicsAgent


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.posture_classifier = pickle.load(open('../joint_control/robot_pose.pkl', 'rb')) #for posture recognition


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        angle = self.perception.joint[joint_name]
        print('get_angle: ' + str(joint_name) + ' ' + str(angle))
        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print('set_angle: ' + str(joint_name), str(angle))
        #self.perception.joint[joint_name] = angle
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        posture = 'unknown'
        # YOUR CODE HERE
        classes = listdir('../joint_control/robot_pose_data')
        posture_data = [self.perception.joint['LHipYawPitch'], self.perception.joint['LHipRoll'], self.perception.joint['LHipPitch'],
                        self.perception.joint['LKneePitch'], self.perception.joint['RHipYawPitch'],
                        self.perception.joint['RHipRoll'], self.perception.joint['RHipPitch'], self.perception.joint['RKneePitch'],
                        self.perception.imu[0], self.perception.imu[1]]
        posture = classes[self.posture_classifier.predict([posture_data])[0]]
        print('get_posture: ' + str(posture))
        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE

        self.keyframes = keyframes
        print('execute_keyframes')




    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print('get_transform')
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print('set_transform: ' + effector_name + ' ' + str(transform))
        self.set_transforms(effector_name, np.array(transform))


if __name__ == '__main__':
    agent = ServerAgent()

    #run RPC Server
    server = SimpleXMLRPCServer(('localhost', 9000), allow_none=True, requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_instance(agent)
    print('server is now available at ' + server.server_address[0])

    try:
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
    except KeyboardInterrupt:
        print('Exiting')

    #agent.keyframes = rightBackToStand()
    agent.run()
    print('KekRobot now running...')

