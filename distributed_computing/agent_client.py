'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''
import threading
from xmlrpc.client import ServerProxy
import weakref
import numpy as np
from joint_control.keyframes import *
from time import sleep

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        self.server = ServerProxy("http://localhost:9000")

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        print('execute_keyframes nonblocking...')
        thread = threading.Thread(target=self.server.execute_keyframes, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        print('set_transform at ' + str(effector_name))
        thread = threading.Thread(target=self.server.set_transform, args=[effector_name, transform.tolist()])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.server = ServerProxy("http://localhost:9000")

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        angle = self.server.get_angle(joint_name)
        print('get_angle: ' + str(joint_name) + ' ' + str(angle))
        return angle
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print('set_angle: ' + str(joint_name), str(angle))
        self.server.set_angle(joint_name, angle)
        #return

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        posture = self.server.get_posture()
        print('get_posture: ' + str(posture))
        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        times = keyframes[1]
        #print(times)
        m = 0  # length of one keyframe animation loop
        for i, time in enumerate(times):
            maxi = max(time)
            if (m < maxi):
                m = maxi
        print('execute_keyframes...')
        self.server.execute_keyframes(keyframes)
        sleep(m + 1)    #block until keyframe completed, +1 because loading keyframe apparently takes time
        #return

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print('get_transform: ' + str(name))
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print('set_transform at ' + str(effector_name))
        self.server.set_transform(effector_name, transform.tolist())
        sleep(5) #block, dont know how I am supposed to know how long
        #return

if __name__ == '__main__':
    agent = ClientAgent()

    # TEST CODE HERE
    print(agent.server.system.listMethods())
    #agent.get_angle('HeadYaw')
    #agent.set_angle('HeadYaw', 1.)
    #agent.get_angle('HeadYaw')
    #agent.execute_keyframes(rightBackToStand())
    #agent.get_posture()
    #T = np.identity(4)
    #T[-1, 1] = 0.05
    #T[-1, 2] = 0.26
    #agent.set_transform('LLeg', T)
    #print(T, type(T))
    #print(agent.get_transform('HeadYaw'))
    #agent.post.execute_keyframes(rightBackToStand())
    #agent.post.set_transform('LLeg', T)




