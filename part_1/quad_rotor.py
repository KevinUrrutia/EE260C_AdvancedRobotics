from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint



class quad_rotor:
    def __init__(self):
        self.l = 0.046 #[m], the arm lengths of the quad_rotor
        self.m = 0.030 #[kg], the mass of the quadrotor
        self.moment_inertia = np.array([[1.43e-5, 0, 0],
                                        [0, 1.43e-5, 0],
                                        [0, 0, 2.89e-5]])
        self.state = np.zeros((12, 1))
        self.Rbw = np.identity(3)
        # self.kp = np.array([[5, 0, 0],
        #                     [0, 5, 0],
        #                     [0, 0, 4.5]]) #5 5 4.5
        self.kp = np.array([[1.5, 0, 0],
                            [0, 1.5, 0],
                            [0, 0, 1.5]]) #5 5 4.5
        self.kd = np.array([[2.5, 0, 0],
                            [0, 2.5, 0],
                            [0, 0, 2.5]]) #10 10 14
        self.kr = np.array([[1000, 0, 0],
                            [0, 1000, 0],
                            [0, 0, 1000]]) #1000 1000 1000
        self.k_omega = np.array([[100, 0, 0],
                                [0, 100, 0],
                                [0, 0, 100]]) #100 100 100
        self.u1 = 0
        self.u2 = np.zeros((3,1))

        self.pre_error_v = np.zeros((3,1))
        self.pre_error_w = np.zeros((3,1))

    def visualization(self, pos):
        #the inputs to the function is the length of the arm of the quad_rotor and the rotation matrix, and the position in the world frame
        fig = plt.figure()
        axs = plt.axes(projection='3d')

        x = np.linspace(0, 12, 100)
        y = np.linspace(0, 12, 100)
        z = np.linspace(0, 12, 100)

        rotor1 = pos + np.dot(np.array([self.l, 0, 0]), self.Rbw)
        rotor2 = pos + np.dot(np.array([0, self.l, 0]), self.Rbw)
        rotor3 = pos + np.dot(np.array([-self.l, 0, 0]), self.Rbw)
        rotor4 = pos + np.dot(np.array([0, -self.l, 0]), self.Rbw)

        rotor1 = rotor1.reshape(3,1)
        rotor2 = rotor2.reshape(3,1)
        rotor3 = rotor3.reshape(3,1)
        rotor4 = rotor4.reshape(3,1)

        x_par1 = np.array([rotor1[0], rotor3[0]])
        y_par1 = np.array([rotor1[1], rotor3[1]])
        z_par1 = np.array([rotor1[2], rotor3[2]])

        x_par2 = np.array([rotor2[0], rotor4[0]])
        y_par2 = np.array([rotor2[1], rotor4[1]])
        z_par2 = np.array([rotor2[2], rotor4[2]])

        axs.axes.set_xlim3d(left=0, right=1)
        axs.axes.set_ylim3d(bottom=0, top=1)
        axs.axes.set_zlim3d(bottom=0, top=1)

        plt.plot(x_par1.flatten(), y_par1.flatten(), z_par1.flatten(), 'bo', linestyle='--')
        plt.plot(x_par2.flatten(), y_par2.flatten(), z_par2.flatten(), 'bo', linestyle='--')

        plt.show()

    def dynamics(self, y0, t):
        #state space X = [x, y, z, dot_x, dot_y, dot_z, phi, theta, psi, angx, angy, angz].T
                        #[0, 1, 2, 3    , 4    , 5    , 6  , 7    , 8  , 9   , 10  , 11]

        R11 = (np.cos(self.state[8])*np.cos(self.state[7])) - (np.sin(self.state[6]*np.sin(self.state[8])*np.sin(self.state[7]))) #c(psi)c(theta) - s(phi)s(psi)s(theta)
        R12 = -np.cos(self.state[6])*np.sin(self.state[8])                                                         #-c(phi)s(psi)
        R13 = (np.cos(self.state[8])*np.sin(self.state[7])) + (np.cos(self.state[7])*np.sin(self.state[6])*np.sin(self.state[8])) #c(psi)s(theta) - c(theta)s(phi)s(psi)

        R21 = (np.cos(self.state[7])*np.sin(self.state[8])) + (np.cos(self.state[8])*np.sin(self.state[6])*np.sin(self.state[7])) #c(theta)s(psi) + c(psi)s(phi)s(theta)
        R22 = np.cos(self.state[6])*np.cos(self.state[8])                                                          #c(phi)c(psi)
        R23 = (np.sin(self.state[8])*np.sin(self.state[7])) - (np.cos(self.state[8])*np.cos(self.state[7])*np.sin(self.state[6])) #s(psi)s(theta) - c(psi)c(theta)s(phi)

        R31 = -np.cos(self.state[6])*np.sin(self.state[7])                                                         #-c(phi)s(theta)
        R32 = np.sin(self.state[6])                                                                           #s(phi)
        R33 = np.cos(self.state[6])*np.cos(self.state[7])                                                          #c(phi)c(theta)

        Rwb = np.array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])

        self.Rbw = Rwb.reshape(3,3)

        gravity_comp = (1/self.m) * np.array([0, 0, -self.m * 9.8]).reshape((3,1))
        thrust_comp = (1/self.m) * np.dot(self.Rbw, np.array([0, 0, self.u1])).reshape((3,1))
        # print("thrust", thrust_comp)
        accel = gravity_comp + thrust_comp
        # print("dyn_accel", accel)

        inv_moment_inertia = np.linalg.inv(self.moment_inertia)
        ang_vel_input = np.array([y0[9], y0[10], y0[11]]).reshape(3,1)
        moment_comp = np.dot(self.moment_inertia, ang_vel_input)


        moment_cross = np.cross(ang_vel_input, moment_comp, axis=0)

        ang_accel = np.dot(inv_moment_inertia,(self.u2.reshape(3,1) - moment_cross))

        ang_vel_matrix = np.array([[np.cos(y0[7]), 0, -np.cos(y0[6])*np.sin(y0[7])],
                                    [0              ,1, np.sin(y0[6])],
                                    [np.sin(y0[7]),0, np.cos(y0[6])*np.cos(y0[7])]], dtype='float')

        ang_vel = np.dot(np.linalg.inv(ang_vel_matrix), np.array([y0[9], y0[10], y0[11]]))


        vel = np.array([y0[3],  y0[4], y0[5]]).reshape(3,1)
        # print(vel)

        state_dot = np.concatenate((vel, accel.reshape(1,3), ang_vel.reshape(1,3), ang_accel.reshape(1,3)), axis=None)
        state_dot = state_dot.astype(float).reshape(-1)
        # print('y0 : ',y0)
        pos = y0[0:3]
        # self.visualization(pos.reshape(1,3))
        return state_dot

    def controller(self, des_pos, des_vel, des_accel, des_yaw, des_yaw_rate):
        pos = np.array([self.state[0], self.state[1], self.state[2]]).reshape(3,1)
        vel = np.array([self.state[3], self.state[4], self.state[5]]).reshape(3,1)

        # print("pos", pos)
        error_pos = pos - des_pos.reshape(3,1)
        print("error_pos", error_pos)
        print("vel",vel)
        error_vel = vel - des_vel.reshape(3,1)
        print(error_vel)

        a_cmd = des_accel.reshape(3,1) - np.dot(self.kd, error_vel) - np.dot(self.kp, error_pos)
        # a_cmd = des_accel - self.PD_pv(error_vel, self.pre_error_v, error_pos, 0.01)
        print("a_cmd",a_cmd)

        u1 = self.m*(a_cmd[2] + 9.8)

        eta = np.array([self.state[6], self.state[7], self.state[8]]).reshape(3,1)

        phi_des = -((a_cmd[1]*np.cos(des_yaw)) - (a_cmd[0]*np.sin(des_yaw)))/(9.8*(np.cos(des_yaw)**2 + np.sin(des_yaw)**2))
        theta_des = a_cmd[0] - (((a_cmd[1]*np.cos(des_yaw)) + (a_cmd[0]*np.sin(des_yaw)))/(9.8*np.cos(des_yaw)))
        eta_des = np.array([phi_des, theta_des, des_yaw]).reshape(3,1)
        error_R = eta - eta_des

        omega = np.array([self.state[9], self.state[10], self.state[11]]).reshape(3,1)
        omega_des = np.array([0, 0, des_yaw_rate]).reshape(3,1)

        error_omega = omega - omega_des

        # u2 = np.dot(np.identity(3), (-np.dot(self.kr, error_R)-np.dot(self.k_omega, error_omega)))
        u2 = np.dot(np.identity(3), -self.PD_rw(error_omega, self.pre_error_w, error_R, 0.01))
        return [u1, u2]

    def PD_pv(self, error_v, pre_error_v, error_p,  dt):

        pout = np.dot(self.kp, error_p)
        dout = np.dot(self.kd, (error_v - pre_error_v)) / dt
        self.pre_error_v = error_v
        return pout + dout

    def PD_rw(self, error_w, pre_error_w, error_r,  dt):

        pout = np.dot(self.k_omega, error_r)
        dout = np.dot(self.k_omega, (error_w - pre_error_w)) / dt
        self.pre_error_w = error_w
        return pout + dout

    def trajectory_planner(self, q0, qh, zt, T):
        #begin at start position q0, that gives desired position and orientation
        x_real = self.state[0]
        y_real = self.state[1]
        z_real = self.state[2]

        x_des = q0[0]
        y_des = q0[1]
        z_des = q0[2]

        self.state[0:3] = q0[0:3]
        self.state[6:9] = q0[3:6]
        pos = self.state[0:3]
        pos = pos.reshape(1,3)
        self.visualization(pos.reshape(1,3))

        #take off
        #discretize the distance
        take_off_time = np.linspace(1, 10, 2)
        delta_pos = zt / len(take_off_time)
        des_pos = self.state[0:3] + np.array([0, 0, delta_pos]).reshape(3,1)
        print("des_pos", des_pos)
        for i in range(len(take_off_time)):
            x_des = des_pos[0]
            y_des = des_pos[1]
            z_des = des_pos[2]

            [self.u1, self.u2] = self.controller(des_pos, np.zeros((3,1)), np.zeros((3, 1)), 0, 0)
            t = np.linspace(0, 1, 2)
            sol = odeint(self.dynamics, self.state.reshape(-1), t)
            self.state = sol[1].reshape(12,1)
            # print(self.state)
            pos = sol[1][0:3]
            print("pos", pos)

            des_pos = des_pos + np.array([0, 0, delta_pos]).reshape(3,1)
            print("des_pos", des_pos)

            x_real = np.append(x_real, pos[0])
            y_real = np.append(y_real, pos[1])
            z_real = np.append(z_real, pos[2])
            self.visualization(pos)

        print("FINISHED TAKEOFF BEGINNING HOVER")

        #hover
        des_pos = self.state[0:3]
        des_vel = np.zeros((3,1))
        des_accel = np.zeros((3,1))
        des_yaw = 0
        des_yaw_rate = 0
        [self.u1, self.u2] = self.controller(self.state[0:3],des_vel, des_accel, 0,  0)
        # print(self.u1, )
        t = np.linspace(0, T, 10)
        sol = odeint(self.dynamics, self.state.reshape(-1), t)
        # print('sol',sol)
        pos = sol[1][0:3]
        print('Hover pos',pos)

        x_des = des_pos[0]
        y_des = des_pos[1]
        z_des = des_pos[2]

        x_real = np.append(x_real, pos[0])
        y_real = np.append(y_real, pos[1])
        z_real = np.append(z_real, pos[2])
        self.visualization(pos.reshape(1,3))

        print("FINISHED HOVER BEGINNING LAND")

        #land
        take_off_time = np.linspace(1, 10, 2)
        delta_pos = zt / len(take_off_time)
        des_pos = self.state[0:3]
        print("start_pos", des_pos)
        for i in range(len(take_off_time)):

            x_des = des_pos[0]
            y_des = des_pos[1]
            z_des = des_pos[2]

            [self.u1, self.u2] = self.controller(des_pos, np.zeros((3,1)), np.zeros((3, 1)), 0, 0)
            t = np.linspace(0, 1, 2)
            sol = odeint(self.dynamics, self.state.reshape(-1), t)
            self.state = sol[1].reshape(12,1)
            # print(self.state)
            pos = sol[1][0:3]
            print("pos", pos)

            des_pos = des_pos - np.array([0, 0, delta_pos]).reshape(3,1)
            print("des_pos", des_pos)

            x_real = np.append(x_real, pos[0])
            y_real = np.append(y_real, pos[1])
            z_real = np.append(z_real, pos[2])
            self.visualization(pos)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(x_real)
        ax1.plot(x_des)

        ax2.plot(y_real)
        ax2.plot(y_des)

        ax3.plot(z_real)
        ax3.plot(z_des)
        plt.show()


def main():
    q0 = np.array([0.5, 0.5, 0.5, 0, 0, 0]).reshape(6,1)
    qh = np.array([1, 1, 1, 0, 0, 0]).reshape(6,1)
    zt = 0.2
    T = 4

    q = quad_rotor()
    q.trajectory_planner(q0, qh, zt, T)


if __name__== "__main__":
    main()
