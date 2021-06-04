'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy import identity
import numpy as np
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        joint_angles = {}
        # YOUR CODE HERE
        #jacobian matrix solution
        lambda0 = 1
        max_step = 0.1
        #print(transform)
        #target_effector = self.chains[effector_name][-1]
        joints = self.chains[effector_name]
        target = np.matrix([self.from_trans(transform)])   #target vector

        theta = np.random.random(len(joints)) * 1e-5
        for i in range(1000):
            self.forward_kinematics(self.perception.joint)  # update joints

            Ts = [0] * len(joints)  #transformation matrices
            i = 0
            for name in joints:
                Ts[i] = self.transforms[name]
                i += 1
            Te = np.matrix([self.from_trans(Ts[-1])]).T #vector of effector point

            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([self.from_trans(i) for i in Ts]).T
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :] # x
            J[1, :] = dT[0, :] # y
            J[-1, :] = 1  # angular
            d_theta = lambda0 * np.linalg.pinv(J) * e
            #print(d_theta)
            theta += np.asarray(d_theta.T)[0]   #angles

            #copy to dictionary
            i = 0
            for joint in joints:
                joint_angles[joint] = theta[i]
                i = i + 1
                if (i == len(theta)):
                    break

            if  np.linalg.norm(d_theta) < 1e-4:
                break

        #print(theta)
        # YOUR CODE HERE
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)

        names = self.chains[effector_name]
        times = [[0.0, 3.0]] * len(names)
        keys = []
        i=0
        for name in names:
            keys.append([[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])
            i+=1

        self.keyframes = (names, times, keys)

    #from lecture but changed to 3d
    def from_trans(self, m):
        '''get x, y, z , theta from transform matrix'''
        x = m[3, 0]
        y = m[3, 1]
        z = m[3, 2]

        theta = 0
        if m[0, 0] == 1: #x rotation
            theta = atan2(m[2, 1], m[1, 1])
        elif m[1, 1] == 1: #y rotation
            theta = atan2(m[0, 2], m[0, 0])
        elif m[2, 2] == 1: #z rotation
            theta = atan2(m[0, 1], m[0, 0])

        return x, y, z, theta


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
