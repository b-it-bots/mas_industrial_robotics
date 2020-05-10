from __future__ import print_function

import math
import random
from PIL import Image, ImageDraw

class Node(object):

    """Docstring for Node. """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.ws = []
        self.probable_ws_direction = ['N', 'S', 'E', 'W']

    def add_ws(self):
        direction = random.choice(self.probable_ws_direction)
        self.probable_ws_direction.remove(direction)
        if direction == 'N':
            if 'S' in self.probable_ws_direction:
                self.probable_ws_direction.remove('S')
            x = random.randint(self.x + 40, self.x + 110)
            y = self.y + 25
            theta = math.pi/2
        if direction == 'S':
            if 'N' in self.probable_ws_direction:
                self.probable_ws_direction.remove('N')
            x = random.randint(self.x + 40, self.x + 110)
            y = self.y + 125
            theta = -math.pi/2
        if direction == 'E':
            if 'W' in self.probable_ws_direction:
                self.probable_ws_direction.remove('W')
            x = self.x + 125
            y = random.randint(self.y + 40, self.y + 110)
            theta = 0.0
        if direction == 'W':
            if 'E' in self.probable_ws_direction:
                self.probable_ws_direction.remove('E')
            x = self.x + 25
            y = random.randint(self.y + 40, self.y + 110)
            theta = math.pi

        self.ws.append({'x': x, 'y': y, 'theta': theta})

    @property
    def num_ws_limit(self):
        limit = 0
        if 'N' in self.probable_ws_direction or 'S' in self.probable_ws_direction:
            limit += 1
        if 'E' in self.probable_ws_direction or 'W' in self.probable_ws_direction:
            limit += 1
        return limit

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        string = '<'
        string += 'x: ' + str(self.x) + ', '
        string += 'y: ' + str(self.y) + ', '
        string += 'prob: ' + str(self.probable_ws_direction) + ', '
        string += 'ws: ' + str(self.ws)
        string += '>'
        return string

class TestClass(object):

    def __init__(self):
        self._grid_dim = 150
        self._num_of_grid = 4
        self._start_cell = (1, 0)
        self._exit_cell = (3, 3)
        self._num_of_ws = 5
        self._grid = [[None for _ in range(self._num_of_grid)] for _ in range(self._num_of_grid)]
        self._walled_edges = []
        self._generate_nodes_and_edges()
        self._grid_map = Image.new('L', (self._num_of_grid*self._grid_dim, self._num_of_grid*self._grid_dim), 255)
        self._draw_obj = ImageDraw.Draw(self._grid_map)

    def _generate_nodes_and_edges(self):
        for i in range(self._num_of_grid):
            for j in range(self._num_of_grid):
                node = Node(x=j*self._grid_dim, y=i*self._grid_dim)
                self._grid[i][j] = node

        all_edges = []
        for i in range(self._num_of_grid-1):
            for j in range(self._num_of_grid):
                all_edges.append(((i, j), (i+1, j)))
        for i in range(self._num_of_grid):
            for j in range(self._num_of_grid-1):
                all_edges.append(((i, j), (i, j+1)))
        walled_edges = []
        for edge in all_edges:
            if random.random() < 0.3:
                walled_edges.append(edge)
        walled_edge = self._make_connected(walled_edges)
        self._walled_edges = walled_edges

        self._generate_ws()

    def _generate_ws(self):
        max_possible_ws = 0
        for i in range(self._num_of_grid):
            for j in range(self._num_of_grid):
                self._calc_ws_lim(i, j)
                max_possible_ws += self._grid[i][j].num_ws_limit
        if max_possible_ws < self._num_of_ws:
            print('ERROR. This many ws not possible with current configuration')
            return

        for i in range(self._num_of_ws):
            while True:
                i, j = random.randint(0, self._num_of_grid-1), random.randint(0, self._num_of_grid-1)
                if len(self._grid[i][j].ws) < self._grid[i][j].num_ws_limit:
                    self._grid[i][j].add_ws()
                    break
        # for i in range(self._num_of_grid):
        #     for j in range(self._num_of_grid):
        #         print(self._grid[i][j])

    def _calc_ws_lim(self, i, j):
        if (i, j) == self._start_cell or (i, j) == self._exit_cell:
            self._grid[i][j].probable_ws_direction = []
            return
        connected_neighbours = self._get_connected_neighbour((i, j), self._walled_edges)
        for neighbour in connected_neighbours:
            diff_i = neighbour[0] - i
            diff_j = neighbour[1] - j
            if diff_i > 0:
                self._grid[i][j].probable_ws_direction.remove('S')
            elif diff_i < 0:
                self._grid[i][j].probable_ws_direction.remove('N')
            if diff_j > 0:
                self._grid[i][j].probable_ws_direction.remove('E')
            elif diff_j < 0:
                self._grid[i][j].probable_ws_direction.remove('W')
        return

    def _make_connected(self, walled_edges):
        connected_nodes = self._get_connected_nodes(walled_edges)
        while len(connected_nodes) != self._num_of_grid**2:
            disconnected_nodes = []
            for i in range(self._num_of_grid):
                for j in range(self._num_of_grid):
                    if (i, j) not in connected_nodes:
                        disconnected_nodes.append((i, j))
            node = random.choice(disconnected_nodes)
            walls_around_node = [walled_edge for walled_edge in walled_edges \
                                 if node == walled_edge[0] or node == walled_edge[1]]
            if len(walls_around_node) == 0:
                continue
            chosen_wall = random.choice(walls_around_node)
            walled_edges.remove(chosen_wall)
            print("Disconnected map. Removed wall", chosen_wall)
            connected_nodes = self._get_connected_nodes(walled_edges)
        return walled_edges

    def _get_connected_nodes(self, walled_edges):
        connected_nodes = []
        fringe = [self._start_cell]
        while len(fringe) > 0:
            current = fringe.pop(0)
            if current in connected_nodes:
                continue
            connected_neighbours = self._get_connected_neighbour(current, walled_edges)
            connected_nodes.append(current)
            for neighbour in connected_neighbours:
                if neighbour not in connected_nodes:
                    fringe.append(neighbour)
        return connected_nodes

    def _get_connected_neighbour(self, node, walled_edges):
        neighbours = self._get_neighbours(*node)
        connected_neighbours = [n for n in neighbours \
                                if (node, n) not in walled_edges and (n, node) not in walled_edges]
        return connected_neighbours

    def _get_neighbours(self, i, j):
        neighbours = []
        if i != 0:
            neighbours.append((i-1, j))
        if i != self._num_of_grid-1:
            neighbours.append((i+1, j))
        if j != 0:
            neighbours.append((i, j-1))
        if j != self._num_of_grid-1:
            neighbours.append((i, j+1))
        return neighbours

    def create_image(self):
        # draw map edges
        length = self._num_of_grid*self._grid_dim
        self._draw_obj.line([(0, 0), (0, length)], fill=0, width=4)
        self._draw_obj.line([(0, 0), (length, 0)], fill=0, width=4)
        self._draw_obj.line([(0, length), (length, length)], fill=0, width=4)
        self._draw_obj.line([(length, 0), (length, length)], fill=0, width=4)
        for edge in self._walled_edges:
            n1 = self._grid[edge[0][0]][edge[0][1]]
            n2 = self._grid[edge[1][0]][edge[1][1]]
            if n1.x == n2.x:
                xy = [(n2.x, n2.y), (n2.x+self._grid_dim, n2.y)]
            else:
                xy = [(n2.x, n2.y), (n2.x, n2.y+self._grid_dim)]
            self._draw_obj.line(xy, fill=0, width=4)
        for i, row in enumerate(self._grid):
            for j, cell in enumerate(row):
                self._draw_cell(self._grid[i][j])

        # drawObject.line([(0, 0), (200, 200)], width=2)
        # drawObject.rectangle([(0, 0), (200, 200)], fill=0, width=0)
        self._grid_map.save("/tmp/map.pgm")

    def _draw_cell(self, node):
        dir_to_px = {'up': [(node.x, node.y), (node.x + self._grid_dim, node.y)],
                     'down': [(node.x, node.y + self._grid_dim), (node.x + self._grid_dim, node.y + self._grid_dim)],
                     'left': [(node.x, node.y), (node.x, node.y + self._grid_dim)],
                     'right': [(node.x + self._grid_dim, node.y), (node.x + self._grid_dim, node.y + self._grid_dim)]}
        print(node)
        for ws in node.ws:
            if ws['theta'] == math.pi or ws['theta'] == 0.0: # 'E' or 'W' facing
                x, y = ws['x'], ws['y']
                xy = [(x-25, y-40), (x+25, y+40)]
            else: # 'N' or 'S' facing
                x, y = ws['x'], ws['y']
                xy = [(x-40, y-25), (x+40, y+25)]
            self._draw_obj.rectangle(xy, outline=0, fill=205, width=2)

if __name__ == "__main__":
    TC = TestClass()
    TC.create_image()
