from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


class quad_rotor:
    def __init__(self):
        self.l = 0.046 #[m], the arm lengths of the quad_rotor
        self.m = 0.030 #[kg], the mass of the quadrotor
        self.moment_inertia = np.array([[1.43e-5, 0, 0],
                                        [0, 1.43e-5, 0],
                                        [0, 0, 2.89e-5]])
        self.state = np.zeros((12, 1))
        self.Rbw = None
        # self.Rbw #rotation matrix in the inertial frame

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

    def dynamics(self, t, u1, u2):
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

        self.Rbw = Rwb.T

        gravity_comp = (np.dot((1/self.m), np.array([0, 0, -self.m * 9.8]))).reshape((3,1))
        thrust_comp = (1/self.m) * np.dot(self.Rbw, np.array([0, 0, u1])).reshape((3,1))
        accel = gravity_comp + thrust_comp

        inv_moment_inertia = np.linalg.inv(self.moment_inertia)
        ang_vel_input = np.array([self.state[9], self.state[10], self.state[11]]).reshape(3,1)
        moment_comp = np.dot(self.moment_inertia, ang_vel_input)


        moment_cross = np.cross(ang_vel_input, moment_comp, axis=0)

        ang_accel = np.dot(inv_moment_inertia,(u2.reshape(3,1) - moment_cross))

        ang_vel_matrix = np.array([[np.cos(self.state[7]), 0, -np.cos(self.state[6])*np.sin(self.state[7])],
                                    [0              ,1, np.sin(self.state[6])],
                                    [np.sin(self.state[7]),0, np.cos(self.state[6])*np.cos(self.state[7])]], dtype='float')

        ang_vel = np.dot(np.linalg.inv(ang_vel_matrix), np.array([self.state[9], self.state[10], self.state[11]]))


        vel = np.array([self.state[3],  self.state[4], self.state[5]]).reshape(3,1)

        state_dot = np.concatenate((vel, accel.reshape(1,3), ang_vel.reshape(1,3), ang_accel.reshape(1,3)), axis=None)
        return state_dot

# def controller(des_pos, des_vel, des_accel, des_yaw, des_yaw_rate):



def main():
    rot_w = np.identity(3)
    moment = np.array([0,0,0])

    qd = quad_rotor()
    qd.dynamics(1, 1 , moment)
    pos = np.array([qd.state[0], qd.state[1], qd.state[1]])
    qd.visualization(pos.reshape(1,3))

if __name__== "__main__":
    main()
