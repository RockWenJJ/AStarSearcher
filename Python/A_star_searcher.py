import numpy as np
import math

class AStarSearcher:
    def __init__(self, start_pt, target_pt, gridmap, type="diagonal"):
        self.gridmap = gridmap
        self.size_x, self.size_y = gridmap.shape
        self.type = type
        self.start_pt = start_pt
        self.target_pt = target_pt
        self.start_idx = self.point2index(start_pt)
        self.target_idx = self.point2index(target_pt)
        self.obstacles = self.get_obstacles()
        self.map_h = self.init_map_h()
        self.map_g = self.init_map_g()
        self.map_parents = self.init_parents()
        self.open_list = []
        self.close_list = []

    def get_distance(self, pt_start, pt_end):
        dx = abs(pt_start[0] - pt_end[0])
        dy = abs(pt_start[1] - pt_end[1])
        if self.type == "diagonal":
            distance = min(dx, dy) * math.sqrt(2) + max(dx, dy) - min(dx, dy)
        elif self.type == "manhattan":
            distance = dx + dy
        elif self.type == "euclidean":
            distance = math.sqrt(dx**2 + dy**2)
        else:
            distance = max(dx, dy)
        return distance

    def point2index(self, pt):
        index = pt[0] * self.size_y + pt[1]
        return index

    def index2point(self, index):
        ptx = index / self.size_y
        pty = index % self.size_y
        pt = (int(ptx), int(pty))
        return pt

    def get_obstacles(self):
        obstacles = []
        for x in range(self.gridmap.shape[0]):
            for y in range(self.gridmap.shape[1]):
                if self.gridmap[x][y] == 3:
                    obstacles.append(self.point2index((x, y)))
        return obstacles

    def init_map_h(self):
        map_h = np.zeros_like(self.gridmap, dtype=np.float32)
        for x in range(self.gridmap.shape[0]):
            for y in range(self.gridmap.shape[1]):
                distance = self.get_distance((x,y), self.target_pt)
                map_h[x][y] = distance + np.random.rand()
        map_h[self.target_pt[0]][self.target_pt[1]] = 0.0
        return map_h

    def init_map_g(self):
        map_g = np.ones_like(self.gridmap, dtype=np.float32) * self.gridmap.shape[0] * self.gridmap.shape[1]
        map_g[self.start_pt[0]][self.start_pt[1]] = 0
        return map_g

    def init_parents(self):
        map_parents = np.ones_like(self.gridmap)*(-1)
        return map_parents

    def insert_open_list(self, pt_index, parent_index):
        pt = self.index2point(pt_index)
        pt_parent = self.index2point(parent_index)

        self.open_list.append(pt_index)
        self.map_parents[pt[0]][pt[1]] = parent_index
        distance = self.get_distance(pt, pt_parent)
        self.map_g[pt[0]][pt[1]] = self.map_g[pt_parent[0]][pt_parent[1]] + distance

    def update_open_list(self, pt_index, parent_index):
        pt = self.index2point(pt_index)
        pt_parent = self.index2point(parent_index)

        distance = self.get_distance(pt, pt_parent)
        tmp_g = self.map_g[pt_parent[0]][pt_parent[1]] + distance
        if self.map_g[pt[0]][pt[1]] > tmp_g:
            self.map_g[pt[0]][pt[1]] = tmp_g
            self.map_parents[pt[0]][pt[1]] = parent_index

    def get_lowest_pt(self):
        fmin = self.gridmap.shape[0] * self.gridmap.shape[1]
        idx_lowest = -1
        for idx, pt_index in enumerate(self.open_list):
            pt = self.index2point(pt_index)
            ptx, pty = pt[0], pt[1]
            h = self.map_h[ptx][pty]
            g = self.map_g[ptx][pty]
            f = h + g
            if f < fmin:
                idx_lowest = idx
                fmin = f
        return idx_lowest

    def get_neighbors(self, pt_index):
        cur_pt = self.index2point(pt_index)
        neighbors = []
        done = False
        if self.type != "diagonal":
            possible_neighbors = [(cur_pt[0]-1, cur_pt[1]),
                                  (cur_pt[0], cur_pt[1]-1),
                                  (cur_pt[0], cur_pt[1]+1),
                                  (cur_pt[0]+1, cur_pt[1])]
        else:
            possible_neighbors = [(x, y) for x in range(cur_pt[0]-1, cur_pt[0]+2) for y in range(cur_pt[1]-1, cur_pt[1]+2)]
        for neighbor in possible_neighbors:
            if neighbor[0] < 0 or neighbor[0] >= self.size_x:
                continue
            if neighbor[1] < 0 or neighbor[1] >= self.size_y:
                continue
            if self.gridmap[neighbor[0]][neighbor[1]] == 3 or \
                    self.gridmap[neighbor[0]][neighbor[1]] == 1:
                continue
            neighbor_index = self.point2index(neighbor)
            if neighbor_index in self.close_list:
                continue
            if neighbor_index == pt_index:
                continue
            if self.gridmap[neighbor[0]][neighbor[1]] == 2:
                done = True
            neighbors.append(neighbor_index)
        return neighbors, done

    def get_path(self):
        self.insert_open_list(self.start_idx, self.start_idx)
        while(len(self.open_list) > 0):
            idx_lowest = self.get_lowest_pt()

            cur_pt_index = self.open_list.pop(idx_lowest)
            self.close_list.append(cur_pt_index)
            cur_neighbors, done = self.get_neighbors(cur_pt_index)
            if not done:
                for neighbor in cur_neighbors:
                    if neighbor in self.open_list:
                        self.update_open_list(neighbor, cur_pt_index)
                    else:
                        self.insert_open_list(neighbor, cur_pt_index)
            else:
                self.map_parents[self.target_pt[0]][self.target_pt[1]] = cur_pt_index
                break

        # back track to get the path
        path_xs = []
        path_ys = []
        cur_index = self.target_idx
        while cur_index != self.start_idx:
            cur_pt = self.index2point(cur_index)
            path_xs.append(cur_pt[0])
            path_ys.append(cur_pt[1])
            cur_index = self.map_parents[cur_pt[0]][cur_pt[1]]

        path_xs.append(self.start_pt[0])
        path_ys.append(self.start_pt[1])

        return path_xs, path_ys