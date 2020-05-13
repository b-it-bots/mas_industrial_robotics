from __future__ import print_function

import os
import yaml
import math
import random
from PIL import Image, ImageDraw

from mir_world_generation.node import Node
from mir_world_generation.utils import Utils

class GridBasedGenerator(object):

    """
    Generate world model for @work arenas based on a grid of cells.
    Each cell in the grid can contain atmost 2 workstations. 
    There is a start and exit cell which will contain no workstations.
    The grid is surrounded with walls on all 4 sides.
    There can be walls between 2 adjacent cells.
    """

    def __init__(self):
        code_dir = os.path.abspath(os.path.dirname(__file__))
        common_dir = os.path.dirname(code_dir)
        config_dir = os.path.join(common_dir, "config")
        config_file = os.path.join(config_dir, "config.yaml")
        with open(config_file, 'r') as file_obj:
            config_data = yaml.safe_load(file_obj)

        # configurable variables
        self._num_of_rows = config_data.get('num_of_rows', 3)
        self._num_of_cols = config_data.get('num_of_cols', 4)
        self._ws_type_to_num = config_data.get('ws_type_to_num', {})
        self._max_retries_allowed = config_data.get('max_retries_allowed', 5)
        self._generation_dir = config_data.get('generation_dir', '/tmp')
        self._base_link_to_ws_center = config_data.get('base_link_to_ws_center', 0.65)
        self._wall_generation_threshold = config_data.get('wall_generation_threshold', 0.3)
        self._noise_threshold = config_data.get('noise_threshold', 0.1)

        # hardcoded values         
        # 0.01 resolution makes 1 px = 1 cm. This makes occ generation calc easy
        self._resolution = 0.01 # meters/pixel
        # 1.5 x 1.5 meter cell is by design since it allows 2 ws and youbot at the same time
        self._grid_dim = int(1.5 / self._resolution) # pixels

        # randomly choose a start cell in first column
        self._start_cell = (random.randint(0, self._num_of_rows-1), 0)
        # randomly choose an exit cell in last column
        self._exit_cell = (random.randint(0, self._num_of_rows-1), self._num_of_cols-1)

        self._grid = [[None for _ in range(self._num_of_cols)] for _ in range(self._num_of_rows)]
        self._walled_edges = []
        self._ws = []

    def generate_configuration(self):
        """
        1. Generate a grid of cells (Node object).
        2. Add walls between these cells randomly.
        3. Add required amount of workstations in this grid randomly.
        4. If required amount of workstations are more that what is possible with 
        the current randomly generated configuration, repeat 1, 2 and 3.
        
        Return false after failing multiple times.

        :returns: bool

        """
        print('='*40)
        attempts = 0
        while attempts < self._max_retries_allowed:
            attempts += 1
            print('Generating wall and ws configuration. Attempt:', attempts)
            for i in range(self._num_of_rows):
                for j in range(self._num_of_cols):
                    self._grid[i][j] = Node(col=j, row=i, size=self._grid_dim)

            all_edges = []
            for i in range(self._num_of_rows-1):
                for j in range(self._num_of_cols):
                    all_edges.append(((i, j), (i+1, j)))
            for i in range(self._num_of_rows):
                for j in range(self._num_of_cols-1):
                    all_edges.append(((i, j), (i, j+1)))
            walled_edges = []
            for edge in all_edges:
                if random.random() < self._wall_generation_threshold:
                    walled_edges.append(edge)
                    print('Adding wall', edge)
            walled_edge = self._make_connected(walled_edges)
            self._walled_edges = walled_edges

            success = self._generate_ws()
            if success:
                print('='*40)
                print()
                return True

            print()
        return False

    def create_occ_grid(self, border=100):
        """
        Create occupancy grid map (map.pgm) and its config (map.yaml) based on
        walls and grid

        :border: int (pixels)

        """
        height = self._num_of_rows*self._grid_dim
        width = self._num_of_cols*self._grid_dim
        self._grid_map = Image.new('L', (width + 2*border, height + 2*border), 205)
        self._draw_obj = ImageDraw.Draw(self._grid_map)

        # draw clear reactangle with walls around it
        xy = [(-2, -2), (width+2, height+2)]
        self._draw_obj.rectangle(GridBasedGenerator.offset_xy_with_border(xy, border),
                                 outline=0, fill=255, width=4)

        # draw internal walls
        for edge in self._walled_edges:
            n1 = self._grid[edge[0][0]][edge[0][1]]
            n2 = self._grid[edge[1][0]][edge[1][1]]
            if n1.x == n2.x:
                xy = [(n2.x, n2.y), (n2.x+self._grid_dim, n2.y)]
            else:
                xy = [(n2.x, n2.y), (n2.x, n2.y+self._grid_dim)]
            self._draw_obj.line(GridBasedGenerator.offset_xy_with_border(xy, border), fill=0, width=4)

        # draw WS
        for ws_dict in self._ws:
            direction = Node.get_direction_from_theta(ws_dict['theta'])
            x, y = ws_dict['x'], ws_dict['y']
            if direction == 'E' or direction == 'W':
                xy = [(x-Node.ws_width/2, y-Node.ws_length/2),
                      (x+Node.ws_width/2, y+Node.ws_length/2)]
            else: # 'N' or 'S' facing
                xy = [(x-Node.ws_length/2, y-Node.ws_width/2),
                      (x+Node.ws_length/2, y+Node.ws_width/2)]
            self._draw_obj.rectangle(
                    GridBasedGenerator.offset_xy_with_border(xy, border),
                    outline=0, fill=205, width=2)
            self._draw_obj.text([(x+border-12, y+border)],
                                ws_dict['type'].upper()+ws_dict['id'],
                                fill=255)

        # add noise
        for i in range(self._grid_map.width):
            for j in range(self._grid_map.height):
                if self._grid_map.getpixel((i, j)) == 0: # px is black
                    # one of the surrounding px is white
                    if self._grid_map.getpixel((i-1, j)) == 255 or\
                       self._grid_map.getpixel((i+1, j)) == 255 or\
                       self._grid_map.getpixel((i, j-1)) == 255 or\
                       self._grid_map.getpixel((i, j+1)) == 255:
                        if random.random() < self._noise_threshold:
                            self._grid_map.putpixel((i, j), 255)

        image_path = os.path.join(self._generation_dir, 'map.pgm')
        self._grid_map.save(image_path)
        print('Occupancy grid map created:', image_path)
        self._save_occ_grid_yaml_file(height, border)

    def create_xacro(self, name="at_work_arena"):
        """
        Create a xacro file which can be spawned into gazebo simulator based on
        walls and grid

        :name: str (name of world and name of xacro file)
        """
        # read snippets
        code_dir = os.path.abspath(os.path.dirname(__file__))
        common_dir = os.path.dirname(code_dir)
        xacro_snippet_dir = os.path.join(common_dir, "xacro_snippet")
        snippets = {}
        for file_name in os.listdir(xacro_snippet_dir):
            with open(os.path.join(xacro_snippet_dir, file_name), 'r') as file_obj:
                snippets[file_name] = file_obj.read()

        file_string = []

        # start with headers
        file_string.append(snippets['beginning'].format(**{'NAME': name}))

        # add walls
        wall_dict_list = self._get_wall_dict_list()
        for wall in wall_dict_list:
            file_string.append(snippets['wall'].format(**wall))

        # add all ws (ws, sh, pp)
        for ws_dict in self._ws:
            x = ws_dict['x']*self._resolution + (self._start_cell[1]*1.5) - 0.75
            y = ws_dict['y']*self._resolution - (self._start_cell[0]*1.5) - 0.75
            file_string.append(snippets[ws_dict['type']].format(
                **{'ID': ws_dict['id'],'X': x, 'Y': -y, 'YAW': ws_dict['theta']}))

        # end with closing tag
        file_string.append(snippets['end'].format())

        # write to a file
        string = ''.join(file_string)
        xacro_path = os.path.join(self._generation_dir, name+'.xacro')
        with open(xacro_path, 'w') as file_obj:
            file_obj.write(string)
        print('Xacro file created:', xacro_path)

    def create_nav_goal(self):
        """
        Create navigation goals file based on ws

        """
        nav_goals = []
        for ws_dict in self._ws:
            x = ws_dict['x']*self._resolution + (self._start_cell[1]*1.5) - 0.75
            y = ws_dict['y']*self._resolution - (self._start_cell[0]*1.5) - 0.75
            theta = ws_dict['theta']
            name = ws_dict['type'].upper() + ws_dict['id']
            delta_x = (math.cos(theta) * -self._base_link_to_ws_center)
            delta_y = (math.sin(theta) * -self._base_link_to_ws_center)
            pos = str([round(x+delta_x, 3), round(-y+delta_y, 3), round(theta, 3)])
            nav_goals.append(name+': '+pos)
        nav_goals.sort()
        nav_goals.append('START: [0.0, 0.0, 0.0]')
        x = (self._exit_cell[1]*1.5) + (self._start_cell[1]*1.5)
        y = -(self._exit_cell[0]*1.5) + (self._start_cell[0]*1.5)
        nav_goals.append('EXIT: [' + str(x) + ', ' + str(y) + ', 0.0]')

        nav_goal_path = os.path.join(self._generation_dir, 'navigation_goals.yaml')
        with open(nav_goal_path, 'w') as file_obj:
            file_obj.write('\n'.join(nav_goals))
        print('Navigation goals file created:', nav_goal_path)

    def _generate_ws(self):
        # calc probable position of workstation for each cell
        max_possible_ws = 0
        for i in range(self._num_of_rows):
            for j in range(self._num_of_cols):
                self._calc_cell_ws_probability(i, j)
                max_possible_ws += self._grid[i][j].remaining_ws_slot

        total_ws_needed = sum(self._ws_type_to_num.values())
        print('Required workstation:', total_ws_needed)
        print('Max possible workstation:', max_possible_ws)
        if max_possible_ws < total_ws_needed:
            print('ERROR. This many ws are not possible with current configuration')
            return False

        # randomly pick a cell and generate a ws in it if possible
        for ws_type, num in self._ws_type_to_num.iteritems():
            for ws_num in range(1, num+1):
                while True:
                    i = random.randint(0, self._num_of_rows-1)
                    j = random.randint(0, self._num_of_cols-1)
                    if self._grid[i][j].remaining_ws_slot > 0:
                        self._grid[i][j].add_ws()
                        ws_dict = self._grid[i][j].ws[-1]
                        ws_dict['id'] = str(ws_num).zfill(2)
                        ws_dict['type'] = ws_type
                        print('Adding', ws_type, ws_num, 'at (', ws_dict['x'], ',',ws_dict['y'], ')')
                        break

        # aggregate all ws
        self._ws = []
        for i in range(self._num_of_rows):
            for j in range(self._num_of_cols):
                self._ws.extend(self._grid[i][j].ws)
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

    def _get_wall_dict_list(self):
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

        return walls

    def _get_ws_dict_list(self):
        ws_dict_list = []
        for i in range(self._num_of_rows):
            for j in range(self._num_of_cols):
                for ws in self._grid[i][j].ws:
                    x = ws['x']*self._resolution + (self._start_cell[1]*1.5) - 0.75
                    y = ws['y']*self._resolution - (self._start_cell[0]*1.5) - 0.75
                    ws_dict_list.append({'X': x, 'Y': -y,
                                         'YAW': ws['theta']})
        return ws_dict_list

    @staticmethod
    def offset_xy_with_border(xy, border):
        return [(xy[0][0]+border, xy[0][1]+border), (xy[1][0]+border, xy[1][1]+border)]

    def _save_occ_grid_yaml_file(self, height, border):
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

        yaml_path = os.path.join(self._generation_dir, 'map.yaml')
        with open(yaml_path, 'w') as file_obj:
            yaml.safe_dump(data, file_obj, default_flow_style=False)
        print('Occupancy grid config created:', yaml_path)

    def _get_x_y_from_2_points(self, p1, p2):
        x = (p1[0] + p2[0])/2 + (self._start_cell[1]*1.5) - 0.75
        y = (p1[1] + p2[1])/2 - (self._start_cell[0]*1.5) - 0.75
        return x, y

if __name__ == "__main__":
    GBG = GridBasedGenerator()
    if GBG.generate_configuration():
        GBG.create_occ_grid()
        GBG.create_xacro()
        GBG.create_nav_goal()
