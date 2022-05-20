from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

from quad_rotor import *

class trajectory_planner:
    # obstacles ([x, y, z], [x, y, z])
    def __init__(self, start, goal, f):
        self.map = []
        self.obstacles = []
        self.start = start
        self.goal = goal
        self.read_txt_file(f)
        # print(self.obstacles)
        fig = plt.figure()
        ax = fig.add_subplot(projection ='3d')
        self.plot_obstacles(fig, ax)
        self.obs_list = self.generate_obstacles()
        self.path = [(1, 5, 2), (1, 5, 3), (1, 4, 3), (0, 4, 2), (0, 3, 1), (1, 2, 0), (2, 1, -1), (3, 0, -1), (4, 1, -2), (5, 0, -1), (6, 0, 0), (7, 0, 0), (8, 1, -1), (9, 2, -1), (10, 1, 0), (11, 1, 1), (12, 1, 2), (13, 1, 3), (14, 2, 3), (15, 3, 3), (16, 4, 3), (17, 5, 3)]
        self.reduced_path = self.waypoint_reduct()
        print(self.reduced_path)
        self.path = self.reduced_path
        self.draw_path(fig, ax)

        self.prev_vel = np.array([0, 0, 0])
        self.prev_wpt = self.start
        self.run()

        plt.show()
        self.quad = quad_rotor()

        ajdghfj
        self.path = self.get_path_from_A_star(self.start, self.goal, self.obs_list)
        print(self.path)
        self.draw_path(fig, ax)
        plt.show()

    #generate the obstacles from the list
    def generate_obstacles(self):
        Nx, Ny, Nz = np.indices((self.map[0], self.map[1], self.map[2]))
        # varray = np.zeros([20, 20, 20])
        obstacles = []
        for i in range(len(self.obstacles)):
            a = (Nx >= self.obstacles[i][0][0]) & (Nx <= self.obstacles[i][0][0] + self.obstacles[i][1][0]) & (Ny >= self.obstacles[i][0][1]) & (Ny <= self.obstacles[i][0][1] + self.obstacles[i][1][1]) & (Nz >= self.obstacles[i][0][2]) & (Nz <= self.obstacles[i][0][2] + self.obstacles[i][1][2])
            obstacles.append(a)
            # varray = varray | obstacle
        voxelarray = obstacles[0] | obstacles[1]
        x, y, z = np.where(voxelarray==1)
        obstacle_list = []
        for i in range(len(x)):
            obstacle_list.append((x[i], y[i], z[i]))
        return(obstacle_list)


        # ax = plt.figure().add_subplot(projection='3d')
        # ax.voxels(voxelarray, facecolors='cyan', edgecolor='k')
        # plt.xlabel('x')
        # plt.ylabel('y')
        # # plt.zlabel('z')
        # plt.show()

        # obstacle1 = (Nx >= 2) & (Nx < 12) & (Ny >= 2) & (Ny < 12) & (Nz >= 0) & (Nz < 10)
        # obstacle2 = (Nx >= 8) & (Nx < 8.1) & (Ny >= 0) & (Ny < 4) & (Nz >= 2) & (Nz < 12)
        # voxelarray = obstacle1 | obstacle2




    #get the plots from the file
    def read_txt_file(self, f):
        for index, line in enumerate(f):
            if (line[:3] == "EOF"):
                break;
            if index == 0: # get the map details
                prev_pos = -2
                for pos, char in enumerate(line):
                    if (char == ";"):
                        number = int(line[prev_pos+2:pos])
                        prev_pos = pos
                        self.map.append(number)
                    elif (char == "."):
                        number = int(line[prev_pos+2:prev_pos+4])
                        self.map.append(number)
                        break;
            if index > 0:
                flag = False
                obstacle = []
                obstacle_start = []
                obstacle_limit = []
                prev_pos = -2
                for pos, char in enumerate(line):
                    if (char == ";"):
                        number = int(line[prev_pos+2:pos])
                        prev_pos = pos
                        obstacle.append(number)
                    if (char == ":"):
                        number = int(line[prev_pos+2:pos])
                        prev_pos = pos
                        obstacle.append(number)
                    if (char == "."):
                        flag = True
                        number = int(line[prev_pos+2:prev_pos+4])
                        obstacle.append(number)
                        break;
                # print(obstacle)
                if flag:
                    for  i in range(3):
                        obstacle_start.append(obstacle[i])
                        obstacle_limit.append(obstacle[i+3])
                    self.obstacles.append((obstacle_start, obstacle_limit))
                    flag = False


    # probably use bar3d because of the 2D plane
    def plot_obstacles(self, fig, ax):
        # fig = plt.figure()
        # ax = fig.add_subplot(projection ='3d')
        # print(self.map)
        ax.set_xlim(0, self.map[0])
        ax.set_ylim(0, self.map[1])
        ax.set_zlim(0, self.map[2])
        x = []
        y = []
        z = []
        dx = []
        dy = []
        dz = []
        for i in range(len(self.obstacles)):
            x.append(self.obstacles[i][0][0])
            y.append(self.obstacles[i][0][1])
            z.append(self.obstacles[i][0][2])
            dx.append(self.obstacles[i][1][0])
            dy.append(self.obstacles[i][1][1])
            dz.append(self.obstacles[i][1][2])
        ax.bar3d(x, y, z, dx, dy, dz, color='cyan')
        plt.xlabel("X")
        plt.ylabel("Y")
        # plt.show()

    def draw_path(self, fig, ax):
        px = []
        py = []
        pz = []
        for i in range(len(self.path)):
            px.append(self.path[i][0])
            py.append(self.path[i][1])
            pz.append(self.path[i][2])
        ax.plot3D(px,py, pz, color='red')


    def neighbors(self, current):
        # neighbors = [(1, 0, 0), (0, 1, 0), (0, 0, 1), (-1, 0, 0), (0, -1, 0), (0, 0, -1),
        #             (1, 1, 1), (1, 1, -1), (1, -1, 1), (-1, 1, 1),
        #             (1, -1, -1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)]
        neighbors = [[1,0,0],[0,0,1],[0,1,0],[-1,0,0],[0,0,-1],[0,-1,0],
                     [1,1,1],[-1,1,1],[1,-1,1],[-1,-1,1],[1,1,-1],[-1,1,-1],
                     [1,-1,-1],[-1,-1,-1],
                     [1,1,0],[0,1,1],[1,0,1],
                     [-1,-1,0],[0,-1,-1],[-1,0,-1],
                     [-1,1,0],[0,1,-1],[-1,0,1],
                     [1,-1,0],[0,-1,1],[1,0,-1]]
        return [(current[0]+nbr[0], current[1]+nbr[1], current[2]+nbr[2]) for nbr in neighbors]

    def heuristic_distance(self, candidate, goal):
        return sqrt((goal[0]-candidate[0])**2 + (goal[1]-candidate[1])**2 + (goal[2]-candidate[2])**2)

    #check for 3d obstacle space - did with dimitris
    def check_3d_obstacle(self, point, obstacles):
        robot_body_dim = 0.046
        for obstacle in obstacles:
            # print(point, " ", obstacle)
            # print(type(obstacle[0]))
            if point[0] >= obstacle[0]-robot_body_dim and point[0] <= obstacle[0]+robot_body_dim + 1:
                if point[1] >= obstacle[1]-robot_body_dim and point[1] <= obstacle[1]+robot_body_dim + 1:
                    if point[2] >= obstacle[2]-robot_body_dim and point[2] <= obstacle[2]+robot_body_dim + 1:
                        return True
        return False

    def get_path_from_A_star(self, start, goal, obstacles):
        # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
        #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
        #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
        # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
        #   note that the path should contain the goal but not the start
        #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

        #check if goal is in obstacles
        # if goal in obstacles:
        if self.check_3d_obstacle(goal, obstacles):
            raise Exception("Goal is in obstacles")

        #open list - all evaluated candidates -- list
        open_list = []
        #close list - all visited candidates -- list/dict
        close_list = []
        #past_cost - cost of all visited candidates following these condition -- dict
            #cost[current] < cost[nbr] and nbr not in obstacles
        past_cost = {}
        #path from start to Goal
        path = []
        #parent
        parent = {}
        #append start to open_list
        open_list.append([0, start])
        #set past_cost of the first element to 0
        past_cost[start] = 0 #can write as past_cost{(start: 0)}
        #set all rest of nodes to infinite
        while open_list: #open_list not empty
            #first node in open_list to current and remove from open_list
            current = open_list.pop(0)[1]
            #add current to close_list
            close_list.append(current)
            #if current is in the goal set, return success and the path to current
            if goal == current:
                # path.append(goal)
                break
            #for nbr in the neighbors
            for candidate in self.neighbors(current):
                #check nbr not in obstacle
                # if (candidate not in obstacles):
                if not self.check_3d_obstacle(candidate, obstacles):
                    #tenative_past_cost = past_cost[current] + cost[current, nbr]
                    new_cost = past_cost[current] + 1
                    #check not in close_list or tenative_past_cost < past_cost[nbr]
                    if (candidate not in close_list) or (new_cost < past_cost[candidate]):
                        #past_cost[candidate] = new_cost
                        past_cost[candidate] = new_cost
                        #get heuristic_distance
                        h = self.heuristic_distance(candidate, goal)
                        #parent[nbr] = current
                        parent[candidate] = current
                        #final cost
                        final = h + past_cost[candidate]
                        #add to open_list
                        open_list.append([final, candidate])
            open_list.sort()

        #go through parents and append it to path and reverse the path list
        #while current is not in start
        current = goal
        while current != start:
            path.append(current)
            current = parent[current]

        #reverse path because it's in goal to start
        path.append(start)
        path.reverse()

        return path

    def waypoint_reduct(self):

        #draw a line between waypoints
        current_waypoint = self.start
        waypoints = [current_waypoint]
        length = len(self.path)
        # print(length)
        break_q = False
        new_idx = self.path.index(current_waypoint)
        new_cnt = 0
        idx = 0
        while current_waypoint != self.goal:
            # current_index = self.path.index(current_waypoint)
            # if idx == length - 2:
            #     current_waypoint = self.goal
            #     waypoints.append(current_waypoint)
            #     print("CURRENT INDEX AT END")
            #     break
            # print(current_index)
            for i in range(len(self.path)):
                wayline = np.linspace(current_waypoint, self.path[i], 50)
                for j in range(len(wayline)):
                    pt = wayline[j]
                    # print(wayline[j][0])
                    if not (self.check_3d_obstacle(pt, self.obs_list)):
                        continue
                    else:
                        current_waypoint = self.path[i-1]
                        # print("FOUND NEW WAYPOINT")
                        # print(current_waypoint)
                        break_q = True
                        break
                if (break_q):
                    # print("BROKE SECOND FOR")
                    break_q = False
                    break
                else:
                    new_cnt = new_cnt + 1

            if idx > length - 1:
                current_waypoint = self.goal
                waypoints.append(current_waypoint)
                # print("CURRENT INDEX AT END")
                break
            # waypoints.append(current_waypoint)
            new_idx = self.path.index(current_waypoint)
            idx = new_idx + new_cnt
            # print(new_idx)
            self.path = self.path[new_idx:len(self.path)]
            print("NEW REDUCED PATH", self.path)
            waypoints.append(current_waypoint)

        return waypoints

    def polynomial_time_scaling_5th_order(self, p_start, v_start, p_end, v_end, T):
        x_array = np.array([p_start, p_end, v_start, v_end, 0, 0]).T
        # print(x_array)

        # T_mat = np.array([(0, 0, 0, 0, 0, 1),
        #         (pow(T, 5), pow(T, 4), pow(T, 3), pow(T, 2), T, 1),
        #         (0, 0, 0, 0, 1, 0),
        #         (5*pow(T, 4), 4*pow(T, 3), 3*pow(T, 2), 2*T, 1), (0, 0, 0, 2, 0, 0),
        #         (20*pow(T, 3), 12*pow(T, 2), 6*T, 2, 0, 0)], dtype=object)
        T_mat = np.array([[0,0,0,0,0,1], [pow(T,5),pow(T,4),pow(T,3), pow(T,2),T,1], [0,0,0,0,1,0],
                        [5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0],
                        [0,0,0,2,0,0], [20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0]])
        # print(T_mat)

        T_mat_inv = np.linalg.inv(T_mat)
        coeff = np.dot(T_mat_inv, x_array)

        return coeff

    def run(self):
        for i in range(len(self.reduced_path)-1):
            self.move_to_point(self.reduced_path[i], self.reduced_path[i+1], i)


    def move_to_point(self, current_wpt, next_wpt, count):
        T = 4

        final_velx = (current_wpt[0] - self.prev_wpt[0]) / T
        final_vely = (current_wpt[1] - self.prev_wpt[1]) / T
        final_velz = (current_wpt[2] - self.prev_wpt[2]) / T

        cx = self.polynomial_time_scaling_5th_order(self.prev_wpt[0], self.prev_vel[0], current_wpt[0], final_velx, T)
        cy = self.polynomial_time_scaling_5th_order(self.prev_wpt[1], self.prev_vel[1], current_wpt[1], final_vely, T)
        cz = self.polynomial_time_scaling_5th_order(self.prev_wpt[2], self.prev_vel[2], current_wpt[2], final_velz, T)

        calc_vx = 0
        calc_vy = 0
        calc_vz = 0

        calc_ax = 0
        calc_ay = 0
        calc_az = 0

        for i in range(T * 10):
            calc_vx = cx[1]  + 2*cx[2]*(i/10) + 3*cx[3]*(i/10)**2 + 4*cx[4]*(i/10)**3 + 5*cx[5]*(i/10)**4
            calc_vy = cx[1]  + 2*cy[2]*(i/10) + 3*cy[3]*(i/10)**2 + 4*cy[4]*(i/10)**3 + 5*cy[5]*(i/10)**4
            calc_vz = cx[1]  + 2*cz[2]*(i/10) + 3*cz[3]*(i/10)**2 + 4*cz[4]*(i/10)**3 + 5*cz[5]*(i/10)**4

            calc_ax = 2*cx[2] + 6*cx[3]*(i/10) + 12*cx[4]*(i/10)**2 + 20*cx[5]*(i/10)**3
            calc_ay = 2*cy[2] + 6*cy[3]*(i/10) + 12*cy[4]*(i/10)**2 + 20*cy[5]*(i/10)**3
            calc_az = 2*cz[2] + 6*cz[3]*(i/10) + 12*cz[4]*(i/10)**2 + 20*cz[5]*(i/10)**3

        #use controller

        self.prev_vel = [calc_vx, calc_vy, calc_vz]
        self.prev_wpt = current_wpt

        print(self.prev_wpt)
        print(self.prev_vel)





    # def calc_point(self, t):
    #     xt = self.a0 + self.a1 * t + self.a2 * t2 + \
    #         self.a3 * t3 + self.a4 * t4 + self.a5 * t5
    #     return xt
    #
    # def calc_first_derivative(self, t):
    #     xt = self.a1 + 2 * self.a2 * t + \
    #         3 * self.a3 * t2 + 4 * self.a4 * t3 + 5 * self.a5 * t4
    #     return xt
    #
    # def calc_second_derivative(self, t):
    #     xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t2 + 20 * self.a5 * t3
    #     return xt
    #
    # #jerk
    # def calc_third_derivative(self, t):
    #     xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t2
    #     return xt


if __name__ == "__main__":
    f = open("SampleEnvironment.txt", "r")
    start = (1, 5, 2)
    goal = (17, 5, 3)
    grid_map = trajectory_planner(start, goal, f)
