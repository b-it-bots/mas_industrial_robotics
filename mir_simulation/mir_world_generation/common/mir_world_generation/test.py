from __future__ import print_function

import random
from PIL import Image, ImageDraw

class Node(object):

    """Docstring for Node. """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        string = '<'
        string += 'x: ' + str(self.x) + ', '
        string += 'y: ' + str(self.y)
        string += '>'
        return string

class TestClass(object):

    def __init__(self):
        self._grid_dim = 100
        self._num_of_grid = 4
        self._start_cell = (1, 0)
        self._exit_cell = (3, 3)
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
            neighbours = self._get_neighbours(*current)
            connected_neighbours = [n for n in neighbours \
                                    if (current, n) not in walled_edges and (n, current) not in walled_edges]
            connected_nodes.append(current)
            for neighbour in connected_neighbours:
                if neighbour not in connected_nodes:
                    fringe.append(neighbour)
        return connected_nodes

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
        # for i, row in enumerate(self._grid):
        #     for j, cell in enumerate(row):
        #         self._draw_cell(self._grid[i][j])

        # drawObject.line([(0, 0), (200, 200)], width=2)
        # drawObject.rectangle([(0, 0), (200, 200)], fill=0, width=0)
        self._grid_map.save("/tmp/map.pgm")

    def _draw_cell(self, node):
        dir_to_px = {'up': [(node.x, node.y), (node.x + self._grid_dim, node.y)],
                     'down': [(node.x, node.y + self._grid_dim), (node.x + self._grid_dim, node.y + self._grid_dim)],
                     'left': [(node.x, node.y), (node.x, node.y + self._grid_dim)],
                     'right': [(node.x + self._grid_dim, node.y), (node.x + self._grid_dim, node.y + self._grid_dim)]}
        # self._draw_obj.rectangle([(node.x, node.y),
        #                           (node.x + self._grid_dim, node.y + self._grid_dim)],
        #                          outline=200, width=1)
        print(node)
        for key in node.walls:
            if node.walls[key]:
                print(key)
                self._draw_obj.line(dir_to_px[key], fill=0, width=4)

if __name__ == "__main__":
    TC = TestClass()
    TC.create_image()
