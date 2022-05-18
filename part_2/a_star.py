# using parts of my code from EE144
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

def plot_obstacles():
    Nx, Ny, Nz = np.indices((20, 20, 20))
    obstacle1 = (Nx >= 2) & (Nx < 12) & (Ny >= 2) & (Ny < 12) & (Nz < 10)
    obstacle2 = (Nx >= 8) & (Nx < 9) & (Ny < 4) & (Nz >= 2) & (Nz < 12)
    voxelarray = obstacle1 | obstacle2
    ax = plt.figure().add_subplot(projection='3d')
    ax.voxels(voxelarray, facecolors='cyan', edgecolor='k')
    plt.xlabel('x')
    plt.ylabel('y')
    # plt.zlabel('z')
    plt.show()

def neighbors(current):
    neighbors = [(-1, 0, 0), (0, -1, 0), (0, 0, -1), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    return [(current[0]+nbr[0], current[1]+nbr[1], current[2]+nbr[2]) for nbr in neighbors]

def heuristic_distance(candidate, goal):
    return sqrt((goal[0]-candidate[0])**2 + (goal[1]-candidate[1])**2 + (goal[2]-candidate[2])**2)

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)]

    #check if goal is in obstacles
    if goal in obstacles:
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
        for candidate in neighbors(current):
            #check nbr not in obstacle
            if (candidate not in obstacles):
                #tenative_past_cost = past_cost[current] + cost[current, nbr]
                new_cost = past_cost[current] + 1
                #check not in close_list or tenative_past_cost < past_cost[nbr]
                if (candidate not in close_list) or (new_cost < past_cost[candidate]):
                    #past_cost[candidate] = new_cost
                    past_cost[candidate] = new_cost
                    #get heuristic_distance
                    h = heuristic_distance(candidate, goal)
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
    path.reverse()

    return path


if __name__ == "__main__":
    start = (0, 0, 0)
    goal = (12, 10, 10)
    obstacles = [(1, 1, 1) , (1, 1, 2), (1, 1, 3), (1, 1, 4), (5, 1, 5), (5, 1, 6), (5, 1, 7), (5, 1, 8)]
    path = get_path_from_A_star(start, goal, obstacles)
    print(path)
    # plot_obstacles()

    