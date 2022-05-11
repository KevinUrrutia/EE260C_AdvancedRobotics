from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class quad_rotor:
    def __init__(self):
        self.l = 0.046 #[m], the arm lengths of the quad_rotor
        self.m = 0.030 #[kg], the mass of the quadrotor
        self.moment_inertia = np.zeros([3, 3])
        self.moment_inertia[0, 0] = 1.43e-05
        self.moment_inertia[1, 1] = 1.43e-05
        self.moment_inertia[2, 2] = 2.89e-05
        self.state = np.zeros((12, 1))
        self.Rbw = np.identity(3)
        self.kp = np.array([[5, 0, 0],
                            [0, 5, 0],
                            [0, 0, 4.5]]) #5 5 4.5
        self.kd = np.array([[10, 0, 0],
                            [0, 10, 0],
                            [0, 0, 14]]) #10 10 14
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

    def visualization(self, pos, fig, axs):
        #the inputs to the function is the length of the arm of the quad_rotor and the rotation matrix, and the position in the world frame
        # fig = plt.figure()
        # axs = plt.axes(projection='3d')
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
        axs.axes.set_xlim3d(left=-pos[0]-4, right=pos[0]+4)
        axs.axes.set_ylim3d(bottom=-pos[1]-4, top=pos[1]+4)
        axs.axes.set_zlim3d(bottom=-pos[2]-4, top=pos[2]+4)
        plt.cla()
        plt.plot(x_par1.flatten(), y_par1.flatten(), z_par1.flatten(), 'bo', linestyle='--')
        plt.plot(x_par2.flatten(), y_par2.flatten(), z_par2.flatten(), 'bo', linestyle='--')
        # plt.show()
        plt.pause(0.01)
        plt.draw()

    def dynamics(self, state, time, q0, qh, zt, Th):
        #state space X = [x, y, z, dot_x, dot_y, dot_z, phi, theta, psi, angx, angy, angz].T
                        #[0, 1, 2, 3    , 4    , 5    , 6  , 7    , 8  , 9   , 10  , 11]
        #Get the desired state
        takeoff = np.zeros([12])
        takeoff[11] = time
        takeoff[2] = qh[2]
        takeoff[1] = qh[1]
        takeoff[0] = qh[0]
        [self.u1, self.u2] = self.controller(state, takeoff[0:3], takeoff[3:6], takeoff[6:9], takeoff[9], takeoff[10], time)
        velocity = state[3:6].reshape(3,1)
        ang_vel = state[9:12].reshape(3,1)
        R11 = (np.cos(state[8])*np.cos(state[7])) - (np.sin(state[6]*np.sin(state[8])*np.sin(state[7]))) #c(psi)c(theta) - s(phi)s(psi)s(theta)
        R12 = -np.cos(state[6])*np.sin(state[8])                                                         #-c(phi)s(psi)
        R13 = (np.cos(state[8])*np.sin(state[7])) + (np.cos(state[7])*np.sin(state[6])*np.sin(state[8])) #c(psi)s(theta) - c(theta)s(phi)s(psi)
        R21 = (np.cos(state[7])*np.sin(state[8])) + (np.cos(state[8])*np.sin(state[6])*np.sin(state[7])) #c(theta)s(psi) + c(psi)s(phi)s(theta)
        R22 = np.cos(state[6])*np.cos(state[8])                                                          #c(phi)c(psi)
        R23 = (np.sin(state[8])*np.sin(state[7])) - (np.cos(state[8])*np.cos(state[7])*np.sin(state[6])) #s(psi)s(theta) - c(psi)c(theta)s(phi)
        R31 = -np.cos(state[6])*np.sin(state[7])                                                         #-c(phi)s(theta)
        R32 = np.sin(state[6])                                                                           #s(phi)
        R33 = np.cos(state[6])*np.cos(state[7])                                                          #c(phi)c(theta)
        R = np.array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])
        accel = ((1 / self.m) * (np.array([0, 0, -self.m * 9.8]).reshape(3,1) + np.dot(R, np.array([0, 0, self.u1])).reshape(3,1))).astype(float)
        ang_accel = np.dot(np.linalg.inv(self.moment_inertia) , self.u2 - np.cross(ang_vel.reshape(1,3), np.dot(self.moment_inertia, ang_vel).reshape(1,3)).reshape(3,1)).astype(float)
        ang_vel_matrix = np.array([[np.cos(state[7]), 0, -np.cos(state[6]) * np.sin(state[7])],
                                         [0, 1, np.sin(state[6])],
                                         [np.sin(state[7]), 0, np.cos(state[6]) * np.cos(state[7])]])
        euler_ang_vel = np.dot(np.linalg.inv(ang_vel_matrix), ang_vel)
        # print(velocity)
        state_dot = np.concatenate((velocity,
                                      accel,
                                      euler_ang_vel,
                                      ang_accel), axis=None)
        return state_dot

    def controller(self, state, des_pos, des_vel, des_accel, des_yaw, des_yaw_rate, t_des):
        self.pre_error_v = np.zeros((3,1))
        self.pre_error_w = np.zeros((3,1))
        pos = np.array([state[0], state[1], state[2]]).reshape(3,1)
        vel = np.array([state[3], state[4], state[5]]).reshape(3,1)
        error_pos = pos - des_pos.reshape(3,1)
        error_vel = vel - des_vel.reshape(3,1)
        a_cmd = des_accel.reshape(3,1) - self.PD_pv(error_vel, self.pre_error_v, error_pos, t_des)
        u1 = self.m*(a_cmd[2] + 9.8)
        eta = np.array([state[6], state[7], state[8]]).reshape(3,1)
        phi_des = -((a_cmd[1]*np.cos(des_yaw)) - (a_cmd[0]*np.sin(des_yaw)))/(9.8*(np.cos(des_yaw)**2 + np.sin(des_yaw)**2))
        theta_des = a_cmd[0] - (((a_cmd[1]*np.cos(des_yaw)) + (a_cmd[0]*np.sin(des_yaw)))/(9.8*np.cos(des_yaw)))
        eta_des = np.array([phi_des, theta_des, des_yaw]).reshape(3,1)
        error_R = eta - eta_des
        omega = np.array([state[9], state[10], state[11]]).reshape(3,1)
        omega_des = np.array([0, 0, des_yaw_rate]).reshape(3,1)
        error_omega = omega - omega_des
        u2 = np.dot(np.identity(3), -self.PD_rw(error_omega, self.pre_error_w, error_R, t_des))
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
        t = np.linspace(0.01, 5, 10)
        state = np.array([q0[0], q0[1], q0[2], 0, 0, 0, q0[3], q0[4], q0[5], 0, 0, 0])
        takeoff = np.array([0, 0, zt, 0, 0, 0])
        sol = odeint(self.dynamics, state, t, args=(q0, takeoff, zt, T))

        t = np.linspace(0.01, 5, 10)
        state = sol[9]
        new_pos = state[0:3] + np.array([2, 0, 0]).reshape(3,1)
        print(new_pos)
        sol = odeint(self.dynamics, state, t, args=(q0, qh, zt, T))
        print(sol[:,0:3])
        fig, ax = plt.subplots()
        ax.set_title("Positioning")
        ax.set_xlabel("t")
        ax.set_ylabel("m")
        ax.plot(t, sol[:,0], label='x')
        ax.plot(t, sol[:,1], label='y')
        ax.plot(t, sol[:,2], label='z')
        ax.grid()
        leg = ax.legend()
        fig, ax = plt.subplots()
        ax.set_title("Linear Velocities")
        ax.set_xlabel("t")
        ax.set_ylabel("m")
        ax.plot(t, sol[:,3], label='x_dot')
        ax.plot(t, sol[:,4], label='y_dot')
        ax.plot(t, sol[:,5], label='z_dot')
        ax.grid()
        leg = ax.legend()
        fig, ax = plt.subplots()
        ax.set_title("Orientation")
        ax.set_xlabel("t")
        ax.set_ylabel("m")
        ax.plot(t, sol[:,6], label='phi')
        ax.plot(t, sol[:,7], label='theta')
        ax.plot(t, sol[:,8], label='psi')
        ax.grid()
        leg = ax.legend()
        fig, ax = plt.subplots()
        ax.set_title("Angular Velocities")
        ax.set_xlabel("t")
        ax.set_ylabel("m")
        ax.plot(t, sol[:,9], label='phi_dot')
        ax.plot(t, sol[:,10], label='theta_dot')
        ax.plot(t, sol[:,11], label='psi_dot')
        ax.grid()
        leg = ax.legend()
        # plt.show()
        fig = plt.figure()
        axs = plt.axes(projection='3d')
        for i in range(sol.shape[0]):
            self.visualization(sol[i][0:3], fig, axs)
        plt.show()
            # plt.close()

def main():
    q0 = np.array([0, 0, 0 , 0, 0, 0])
    qh = np.array([1, 1, 1, 0, 0, 0])
    zt = 1
    T = 0.1
    q = quad_rotor()
    q.trajectory_planner(q0, qh, zt, T)
if __name__== "__main__":
    main()
