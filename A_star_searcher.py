import numpy as np

class AStarSearcher:
    def __init__(self, start_pt, target_pt, gridmap):
        self.start_pt = start_pt
        self.target_pt = target_pt
        self.gridmap = gridmap
        self.size_x, self.size_y = gridmap.shape
        self.obstacles = self.get_obstacles()
        self.map_h = self.init_map_h()
        self.map_g = self.init_map_g()
        self.map_parents = self.init_parents()
        self.open_list = []
        self.close_list = []

    def get_obstacles(self):
        obstacles = []
        for x in range(self.gridmap.shape[0]):
            for y in range(self.gridmap.shape[1]):
                if self.gridmap[x][y] == 3:
                    obstacles.append((x, y))
        return obstacles

    def init_map_h(self):
        map_h = np.zeros_like(self.gridmap, dtype=np.float32)
        for x in range(self.gridmap.shape[0]):
            for y in range(self.gridmap.shape[1]):
                dx = abs(self.target_pt[0] - x)
                dy = abs(self.target_pt[1] - y)
                map_h[x][y] = dx + dy + np.random.rand()
        map_h[self.target_pt[0]][self.target_pt[1]] = 0.0
        return map_h

    def init_map_g(self):
        map_g = np.ones_like(self.gridmap, dtype=np.float32) * self.gridmap.shape[0] * self.gridmap.shape[1]
        map_g[self.start_pt[0]][self.start_pt[1]] = 0
        return map_g

    def init_parents(self):
        map_parents = np.ones_like(self.gridmap)*-1
        return map_parents

    def insert_open_list(self, pt, pt_parent):
        self.open_list.append(pt)
        parent_index = pt_parent[0] * self.size_y + pt_parent[1]
        self.map_parents[pt[0]][pt[1]] = parent_index
        self.map_g[pt[0]][pt[1]] = self.map_g[pt_parent[0]][pt_parent[1]] + 1

    def update_open_list(self, pt, pt_parent):
        return

    def get_lowest_pt(self):
        fmin = self.gridmap.shape[0] * self.gridmap.shape[1]
        idx_lowest = 0
        for idx, pt in enumerate(self.open_list()):
            ptx, pty = pt[0], pt[1]
            h = self.map_h[ptx][pty]
            g = self.map_g[ptx][pty]
            f = h + g
            if f < fmin:
                idx_lowest = idx
                fmin = f
        return idx_lowest

    def get_neighbors(self, cur_pt):
        neighbors = []
        done = False
        possible_neighbors = [(cur_pt[0]-1, cur_pt[1]),
                              (cur_pt[0], cur_pt[1]-1),
                              (cur_pt[0], cur_pt[1]+1),
                              (cur_pt[0]+1, cur_pt[1])]
        for neighbor in possible_neighbors:
            if neighbor[0] < 0 or neighbor[0] >= self.size_x:
                continue
            if neighbor[1] < 0 or neighbor[1] >= self.size_y:
                continue
            if self.gridmap[neighbor[0]][neighbor[1]] == 3 or \
                    self.gridmap[neighbor[0]][neighbor[1]] == 1:
                continue
            if self.gridmap[neighbor[0]][neighbor[1]] == 2:
                done = True
            neighbors.append(neighbor)
        return neighbors, done

    def get_path(self):
        self.insert_open_list(self.start_pt, self.start_pt)
        while(len(self.open_list) > 0):
            idx_lowest = self.get_lowest_pt()
            cur_pt = self.open_list[idx_lowest]

            self.open_list.pop(idx_lowest)
            cur_neighbors, done = self.get_neighbors(cur_pt)
            if not done:
                for neighbor in cur_neighbors:
                    if neighbor in self.open_list:
                        self.update_open_list(neighbor, cur_pt)
                    else:
                        self.insert_open_list(neighbor, cur_pt)
            else:
                parent_idx = cur_pt[0] * self.size_y + cur_pt[1]
                self.map_parents[self.target_pt[0]][self.target_pt[1]] = parent_idx
                break
        path = []






class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def update_parent(self, pNode):
        self.parent = pNode

def get_obstacles(map):
    obstacles = None
    return obstacles

def init_map_h(target_pt, map):
    map_h = np.zeros_like(map)
    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            dx = abs(target_pt[0] - x)
            dy = abs(target_pt[1] - y)
            map_h[x][y] = dx + dy
    return map_h

def init_map_g(map):
    map_g = np.ones_like(map) * map.shape[0] * map.shape[1]
    return map_g

def insert_open(pt, parent_pt, parents):
    parent_idx = parent_pt

    open_list.append(pt)
    return


def A_star_search(start_pt, target_pt, map):
    obstacles = get_obstacles(map)
    map_h = init_map_h(target_pt, map)
    map_g = init_map_g(map)
    open_list = []
    parents = np.ones_like(map)
    insert_open(start_pt, start_pt, parents, open_list)
    while(len(open_list) > 0):
        pt = get_node_with_lowest_f(open_list)  # also pop
        neighbors, done = get_pt_neighbors(pt)
        if done:
            break
        for neighbor in neighbors:
            if neighbor in open_list:
                update_node()
            else:
                insert_open(neighbor, pt)

    paths = []
    return paths