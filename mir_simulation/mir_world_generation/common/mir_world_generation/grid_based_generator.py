from __future__ import print_function

import os
import yaml
import math
import random
from PIL import Image, ImageDraw

from mir_world_generation.node import Node
from mir_world_generation.utils import Utils

class GridBasedGenerator(object):

    def __init__(self):
        self._resolution = 0.01 # meters/pixel
        self._grid_dim = int(1.5 / self._resolution) # pixels
        self._num_of_rows = 3
        self._num_of_cols = 4
        self._start_cell = (random.randint(0, self._num_of_rows-1), 0)
        # self._start_cell = (0, 0)
        self._exit_cell = (random.randint(0, self._num_of_rows-1), self._num_of_cols-1)
        self._ws_type_to_num = {'ws': 8, 'sh': 2}
        self._num_of_ws = sum(self._ws_type_to_num.values())
        self._grid = [[None for _ in range(self._num_of_cols)] for _ in range(self._num_of_rows)]
        self._walled_edges = []

    def generate_configuration(self):
        attempts = 0
        max_retries_allowed = 5
        while attempts < max_retries_allowed:
            attempts += 1
            print()
            print('Generating wall and ws configuration. Attempt:', attempts)
            for i in range(self._num_of_rows):
                for j in range(self._num_of_cols):
                    self._grid[i][j] = Node(x=j*self._grid_dim, y=i*self._grid_dim)

            all_edges = []
            for i in range(self._num_of_rows-1):
                for j in range(self._num_of_cols):
                    all_edges.append(((i, j), (i+1, j)))
            for i in range(self._num_of_rows):
                for j in range(self._num_of_cols-1):
                    all_edges.append(((i, j), (i, j+1)))
            walled_edges = []
            for edge in all_edges:
                if random.random() < 0.3:
                    walled_edges.append(edge)
                    print('Adding wall', edge)
            walled_edge = self._make_connected(walled_edges)
            self._walled_edges = walled_edges

            success = self._generate_ws()
            if success:
                return True
        return False

    def _generate_ws(self):
        max_possible_ws = 0
        for i in range(self._num_of_rows):
            for j in range(self._num_of_cols):
                self._calc_cell_ws_probability(i, j)
                max_possible_ws += self._grid[i][j].remaining_ws_slot
        print('Max possible workstation:', max_possible_ws)
        if max_possible_ws < self._num_of_ws:
            print('ERROR. This many ws not possible with current configuration')
            return False

        for ws_num in range(self._num_of_ws):
            while True:
                i, j = random.randint(0, self._num_of_rows-1), random.randint(0, self._num_of_cols-1)
                if self._grid[i][j].remaining_ws_slot > 0:
                    self._grid[i][j].add_ws()
                    print('Adding ws', ws_num)
                    break
        return True

    def _calc_cell_ws_probability(self, i, j):
        if (i, j) == self._start_cell or (i, j) == self._exit_cell:
            self._grid[i][j].probable_ws_direction = []
            return
        connected_neighbours = Utils.get_connected_neighbour(
                (i, j), self._walled_edges, self._num_of_rows, self._num_of_cols)
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
        connected_nodes = Utils.get_connected_nodes(self._start_cell, walled_edges,
                                                    self._num_of_rows, self._num_of_cols)
        while len(connected_nodes) != self._num_of_rows*self._num_of_cols:
            disconnected_nodes = []
            for i in range(self._num_of_rows):
                for j in range(self._num_of_cols):
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
            connected_nodes = Utils.get_connected_nodes(self._start_cell, walled_edges,
                                                        self._num_of_rows, self._num_of_cols)
        return walled_edges

    def create_image(self):
        height = self._num_of_rows*self._grid_dim
        width = self._num_of_cols*self._grid_dim
        border = 100 # pixel
        self._grid_map = Image.new('L', (width + 2*border, height + 2*border), 205)
        self._draw_obj = ImageDraw.Draw(self._grid_map)

        xy = [(-2, -2), (width+2, height+2)]
        self._draw_obj.rectangle(GridBasedGenerator.offset_xy_with_border(xy, border),
                                 outline=0, fill=255, width=4)

        # draw walls
        for edge in self._walled_edges:
            n1 = self._grid[edge[0][0]][edge[0][1]]
            n2 = self._grid[edge[1][0]][edge[1][1]]
            if n1.x == n2.x:
                xy = [(n2.x, n2.y), (n2.x+self._grid_dim, n2.y)]
            else:
                xy = [(n2.x, n2.y), (n2.x, n2.y+self._grid_dim)]
            self._draw_obj.line(GridBasedGenerator.offset_xy_with_border(xy, border), fill=0, width=4)

        # draw WS
        for i, row in enumerate(self._grid):
            for j, cell in enumerate(row):
                self._draw_cell(self._grid[i][j], border)

        self._grid_map.save("/tmp/map.pgm")
        self._save_yaml_file(height, border)

    @staticmethod
    def offset_xy_with_border(xy, border):
        return [(xy[0][0]+border, xy[0][1]+border), (xy[1][0]+border, xy[1][1]+border)]

    def _draw_cell(self, node, border):
        for ws in node.ws:
            direction = Node.get_direction_from_theta(ws['theta'])
            if direction == 'E' or direction == 'W':
                x, y = ws['x'], ws['y']
                xy = [(x-25, y-40), (x+25, y+40)]
            else: # 'N' or 'S' facing
                x, y = ws['x'], ws['y']
                xy = [(x-40, y-25), (x+40, y+25)]
            self._draw_obj.rectangle(GridBasedGenerator.offset_xy_with_border(xy, border),
                                     outline=0, fill=205, width=2)

    def _save_yaml_file(self, height, border):
        i, j = self._start_cell
        x = j*1.5 + border*self._resolution + 0.75
        y = (height*self._resolution) - i*1.5 + border*self._resolution - 0.75
        data = dict(
            image='map.pgm',
            resolution=0.01,
            origin=[-x, -y, 0.0],
            negate=0,
            occupied_thresh=0.65,
            free_thresh=0.196
        )

        with open('/tmp/map.yaml', 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

    def create_xacro(self, name="test"):
        code_dir = os.path.abspath(os.path.dirname(__file__))
        common_dir = os.path.dirname(code_dir)
        xacro_snippet_dir = os.path.join(common_dir, "xacro_snippet")
        snippets = {}
        for file_name in ['beginning', 'end', 'wall', 'ws', 'sh']:
            with open(os.path.join(xacro_snippet_dir, file_name), 'r') as file_obj:
                snippets[file_name] = file_obj.read()

        file_string = []
        file_string.append(snippets['beginning'].format(**{'NAME': name}))

        walls = []
        wall_id = 1
        # add internal walls
        for edge in self._walled_edges:
            n1 = self._grid[edge[0][0]][edge[0][1]]
            n2 = self._grid[edge[1][0]][edge[1][1]]
            if n1.x == n2.x:
                p1 = (n2.x*self._resolution, n2.y*self._resolution)
                p2 = ((n2.x+self._grid_dim)*self._resolution, n2.y*self._resolution)
                theta = 0.0
            else:
                p1 = (n2.x*self._resolution, n2.y*self._resolution)
                p2 = (n2.x*self._resolution, (n2.y+self._grid_dim)*self._resolution)
                theta = math.pi/2
            x, y = self._get_x_y_from_2_points(p1, p2)
            walls.append({'WALL_ID': str(wall_id).zfill(2), 'LENGTH': 1.5,
                          'X': x, 'Y': -y, 'YAW': theta})
            wall_id += 1

        # add border walls
        height = self._num_of_rows*1.5
        width = self._num_of_cols*1.5
        border_wall_data = [[(0.0, 0.0), (width, 0.0), 0.0],
                            [(0.0, height), (width, height), 0.0],
                            [(0.0, 0.0), (0.0, height), math.pi/2],
                            [(width, 0.0), (width, height), math.pi/2]]
        for p1, p2, theta in border_wall_data:
            x, y = self._get_x_y_from_2_points(p1, p2)
            length = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
            walls.append({'WALL_ID': str(wall_id).zfill(2), 'LENGTH': length,
                          'X': x, 'Y': -y, 'YAW': theta})
            wall_id += 1

        for wall in walls:
            file_string.append(snippets['wall'].format(**wall))

        # add ws
        ws_id = 1
        ws_dict_list = []
        for i in range(self._num_of_rows):
            for j in range(self._num_of_cols):
                for ws in self._grid[i][j].ws:
                    x = ws['x']*self._resolution + (self._start_cell[1]*1.5) - 0.75
                    y = ws['y']*self._resolution - (self._start_cell[0]*1.5) - 0.75
                    ws_dict_list.append({'ID': str(ws_id).zfill(2),
                                         'X': x, 'Y': -y,
                                         'YAW': ws['theta']})
                    ws_id += 1

        random.shuffle(ws_dict_list)
        for ws_type, num in self._ws_type_to_num.iteritems():
            i = num
            while i > 0:
                i -= 1
                ws_dict = ws_dict_list.pop()
                file_string.append(snippets[ws_type].format(**ws_dict))
                

        file_string.append(snippets['end'].format())

        string = ''.join(file_string)
        with open('/tmp/test.xacro', 'w') as file_obj:
            file_obj.write(string)

    def _get_x_y_from_2_points(self, p1, p2):
        x = (p1[0] + p2[0])/2 + (self._start_cell[1]*1.5) - 0.75
        y = (p1[1] + p2[1])/2 - (self._start_cell[0]*1.5) - 0.75
        return x, y

if __name__ == "__main__":
    TC = GridBasedGenerator()
    if TC.generate_configuration():
        TC.create_image()
        TC.create_xacro()
