from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


class quad_rotor:
    def __init__(self):
        self.l = 0.046 #[m], the arm lengths of the quad_rotor
        self.m = 0.030 #[kg], the mass of the quadrotor
        # self.Rbw #rotation matrix in the inertial frame

    def visualization(self, rot_w, pos):
        #the inputs to the function is the length of the arm of the quad_rotor and the rotation matrix, and the position in the world frame
        fig = plt.figure()
        axs = plt.axes(projection='3d')

        x = np.linspace(0, 12, 100)
        y = np.linspace(0, 12, 100)
        z = np.linspace(0, 12, 100)

        rotor1 = pos + np.dot(np.array([self.l, 0, 0]), rot_w)
        rotor2 = pos + np.dot(np.array([0, self.l, 0]), rot_w)
        rotor3 = pos + np.dot(np.array([-self.l, 0, 0]), rot_w)
        rotor4 = pos + np.dot(np.array([0, -self.l, 0]), rot_w)

        x_par1 = [rotor1[0], rotor3[0]]
        y_par1 = [rotor1[1], rotor3[1]]
        z_par1 = [rotor1[2], rotor3[2]]

        x_par2 = [rotor2[0], rotor4[0]]
        y_par2 = [rotor2[1], rotor4[1]]
        z_par2 = [rotor2[2], rotor4[2]]

        axs.axes.set_xlim3d(left=0, right=1)
        axs.axes.set_ylim3d(bottom=0, top=1)
        axs.axes.set_zlim3d(bottom=0, top=1)

        plt.plot(x_par1, y_par1, z_par1, 'bo', linestyle='--')
        plt.plot(x_par2, y_par2, z_par2, 'bo', linestyle='--')

        plt.show()

    def dynamics(self, t, state, u1, u2):
        #state space X = [x, y, z, dot_x, dot_y, dot_z, phi, theta, psi, angx, angy, angz].T
                        #[0, 1, 2, 3    , 4    , 5    , 6  , 7    , 8  , 9   , 10  , 11]
        R11 = (np.cos(state[8])*np.cos(state[7])) - (np.sin(state[6]*np.sin(state[8])*np.sin(state[7]))) #c(psi)c(theta) - s(phi)s(psi)s(theta)
        R12 = -np.cos(state[6])*np.sin(state[8])                                                         #-c(phi)s(psi)
        R13 = (np.cos(state[8])*np.sin(state[7])) + (np.cos(state[7])*np.sin(state[6])*np.sin(state[8])) #c(psi)s(theta) - c(theta)s(phi)s(psi)

        R21 = (np.cos(state[7])*np.sin(state[8])) + (np.cos(state[8])*np.sin(state[6])*np.sin(state[7])) #c(theta)s(psi) + c(psi)s(phi)s(theta)
        R22 = np.cos(state[6])*np.cos(state[8])                                                          #c(phi)c(psi)
        R23 = (np.sin(state[8])*np.sin(state[7])) - (np.cos(state[8])*np.cos(state[7])*np.sin(state[6])) #s(psi)s(theta) - c(psi)c(theta)s(phi)

        R31 = -np.cos(state[6])*np.sin(state[7])                                                         #-c(phi)s(theta)
        R32 = np.sin(state[6])                                                                           #s(phi)
        R33 = np.cos(state[6])*np.cos(state[7])                                                          #c(phi)c(theta)

        Rwb = np.array([[R11, R12, R13],
                        [R21, R22, R23],
                        [R31, R32, R33]])

        Rbw = Rwb.T

        gravity_comp = (np.dot((1/self.m), np.array([0, 0, -self.m * 9.8]))).reshape((3,1))
        thrust_comp = (1/self.m) * np.dot(Rbw, np.array([0, 0, u1])).reshape((3,1))
        accel = gravity_comp + thrust_comp

        print(accel)





def main():
    rot_w = np.identity(3)
    pos = np.array([0,0,0])

    s_0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T

    qd = quad_rotor()
    qd.dynamics(1, s_0, 1 , 1)
    qd.visualization(rot_w, pos)

if __name__== "__main__":
    main()
