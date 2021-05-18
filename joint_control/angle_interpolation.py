'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import *
import numpy as np
from scipy.interpolate import *
import matplotlib.pyplot as plt


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #print(self.target_joints)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        time = perception.time

        for l in range(len(keyframes[0])):

            joint = keyframes[0][l]
            times = (keyframes[1][l])
            angles = [row[0] for row in keyframes[2][l]]
            max_anim_time = max(times)
            cur_time = (time % max_anim_time)
            #yp = np.asarray([row[1][2] for row in keyframes[2][l]])

            #Cubic Spline using scipy
            #last and first point must be identical
            times_id = times + [times[-1]+1.0]
            angles_id = angles + [angles[0]]
            spline = CubicSpline(times_id, angles_id, bc_type="periodic")

            target_joints[joint] = spline(cur_time + 0.001)


            ##plot the shit
            '''x = [(i / 100) * times_id[-1] for i in range(0, 100)]
            y = [spline((i / 100) * times_id[-1]) for i in range(0, 100)]

            plt.plot(x, y, '--')
            plt.plot(times_id, angles_id, 'o')
            plt.show()'''

            '''for k in range(len(times)):
                if(time%times[-1] <= times[k]):
                    # hermite cubic spline interpolation
                    x = times.copy()
                    y = angles.copy()

                    difq = float(y[k] - y[k-1]) / (x[k] - x[k-1])
                    tmp = np.zeros(2)
                    tmp[0] = float(difq - yp[k-1]) / (x[k] - x[k-1])
                    tmp[1] = float(yp[k] - difq) / (x[k] - x[k-1])
                    letzter = (tmp[1] - tmp[0]) / (x[k] - x[k-1])
                    # -----polynom erstellen-----
                    p = np.poly1d([y[k-1]])  # x^0
                    p += yp[k] * np.poly1d([x[k-1]], True)  # x^1
                    p += tmp[0] * np.poly1d([x[k-1], x[k-1]], True)  # x^2
                    p += letzter * np.poly1d([x[k-1], x[k-1], x[k]], True)  # x^3

                    target_joints[joint] = np.polyval(p, (time%times[-1]))
                    #target_joints[joint] = angles[l - 1] + (angles[l] - angles[l - 1]) * (1 / ((times[l] - times[l - 1]) / ((time % times[-1]) - times[l - 1])))
                    break'''

        if "LHipYawPitch" in target_joints:
            target_joints["RHipYawPitch"] = target_joints["LHipYawPitch"]
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    #print(agent.keyframes[2][0])
    #print(agent.keyframes)
    agent.run()
