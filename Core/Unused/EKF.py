#!/usr/bin/env python

'''

Extended Kalman Filter Implementation

'''

from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot

class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer

        a, x, y, v, w, theta, time = symbols(
            'a, x, y, v, w, theta, t')
        d = v*time
        beta = (d/w)*sympy.tan(a)
        r = w/sympy.tan(a)

        # save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v:0, a:0,
                     time:dt, w:wheelbase, theta:0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta

    def predict(self, xjac, ujac, m, u=0):
        #motion model 
        self.x = self.move(self.x, u, self.dt)
        #covariance
        self.P = dot(xjac, self.P).dot(xjac.T) + dot(ujac, m).dot(ujac.T)

    def H_of(self, x, landmark_pos, rng):

        px = landmark_pos[0]
        py = landmark_pos[1]
        hyp = rng**2
        dist = rng

        H = np.array([[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
                      [(py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
        return H

    def Hx(self, x, landmark_pos):

        px = landmark_pos[0]
        py = landmark_pos[1]
        Hx = np.array([[(np.sqrt((x[0, 0] - px)**2 + (x[1, 0] - py)**2))],
                    [(math.atan2(py - x[1, 0], px - x[0, 0])) - x[2, 0]]])

        #WRAP BETWEEN -pi AND pi
        Hx[1] = Hx[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if Hx[1] > np.pi:             # move to [-pi, pi)
            Hx[1] -= 2 * np.pi

        return Hx

    def residual(self, a, b):
        """ compute residual (a-b) between measurements containing
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        #WRAP BETWEEN -pi AND pi
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

    def ekf_update(ekf, z, landmark, rng):
        ekf.update(z, HJacobian=self.H_of, Hx=self.Hx,
                   residual=self.residual,
                   args=(landmark, rng), hx_args=(landmark))
