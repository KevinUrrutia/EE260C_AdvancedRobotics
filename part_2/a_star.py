from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

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
        print(self.waypoint_reduct())
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
                wayline = np.linspace(current_waypoint, self.path[i], 10)
                for j in range(len(wayline)):
                    pt = wayline[j]
                    # print(wayline[j][0])
                    if not (self.check_3d_obstacle(pt, self.obs_list)):
                        continue
                    else:
                        current_waypoint = self.path[i-1]
                        print("FOUND NEW WAYPOINT")
                        print(current_waypoint)
                        break_q = True
                        break
                if (break_q):
                    print("BROKE SECOND FOR")
                    break_q = False
                    break
                else:
                    new_cnt = new_cnt + 1

            idx = new_idx + new_cnt
            if idx > length - 2:
                current_waypoint = self.goal
                waypoints.append(current_waypoint)
                print("CURRENT INDEX AT END")
                break
            new_idx = self.path.index(current_waypoint)
            # idx = new_idx + new_cnt
            print(new_idx)
            self.path = self.path[idx:len(self.path)]
            print("NEW REDUCED PATH", self.path)
            waypoints.append(current_waypoint)

        return waypoints


if __name__ == "__main__":
    f = open("SampleEnvironment.txt", "r")
    start = (1, 5, 2)
    goal = (17, 5, 3)
    grid_map = trajectory_planner(start, goal, f)
