import numpy as np
import matplotlib.pyplot as plt
from A_star_searcher import *

grid_x, grid_y = 10, 10

start_pt = (0,0)
target_pt = (9,9)
type = "diagonal"

#1-start_pt, 2-target_pt, 3-obstacles
map = np.zeros((grid_x, grid_y))
map[start_pt[0]][start_pt[1]] = 1
map[target_pt[0]][target_pt[1]] = 2
obs_x = []
obs_y = []
#initiate obstacles
obs_ratio = 0.25
for x_id in range(grid_x):
    for y_id in range(grid_y):
        if np.random.rand() < obs_ratio and map[x_id][y_id] == 0:
            map[x_id][y_id] = 3
            obs_x.append(x_id)
            obs_y.append(y_id)

a_star_searcher = AStarSearcher(start_pt, target_pt, map, type)
path_xs, path_ys = a_star_searcher.get_path()

offset = 0.5
# plot results
plt.figure()
plt.plot(start_pt[0]+offset, start_pt[1]+offset, "r*")
plt.plot(target_pt[0]+offset, target_pt[1]+offset, "b*")
plt.plot(np.array(obs_x)+offset, np.array(obs_y)+offset, "ko")
plt.plot(np.array(path_xs)+offset, np.array(path_ys)+offset, "g.")
xticks = range(0, grid_x, 1)
yticks = range(0, grid_y, 1)
plt.xticks(xticks)
plt.yticks(yticks)
plt.grid()
plt.show()
