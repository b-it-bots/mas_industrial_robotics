import math
import random

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
            x = self._resolve_overlap(direction) if len(self.ws) > 0 \
                    else random.randint(self.x + 40, self.x + 110)
            y = self.y + 25
            theta = math.pi/2
        if direction == 'S':
            if 'N' in self.probable_ws_direction:
                self.probable_ws_direction.remove('N')
            x = self._resolve_overlap(direction) if len(self.ws) > 0 \
                    else random.randint(self.x + 40, self.x + 110)
            y = self.y + 125
            theta = -math.pi/2
        if direction == 'E':
            if 'W' in self.probable_ws_direction:
                self.probable_ws_direction.remove('W')
            x = self.x + 125
            y = self._resolve_overlap(direction) if len(self.ws) > 0 \
                    else random.randint(self.y + 40, self.y + 110)
            theta = 0.0
        if direction == 'W':
            if 'E' in self.probable_ws_direction:
                self.probable_ws_direction.remove('E')
            x = self.x + 25
            y = self._resolve_overlap(direction) if len(self.ws) > 0 \
                    else random.randint(self.y + 40, self.y + 110)
            theta = math.pi

        self.ws.append({'x': x, 'y': y, 'theta': theta})

    def _resolve_overlap(self, direction):
        ws1 = self.ws[0] # already present WS
        ws1_dir = Node.get_direction_from_theta(ws1['theta'])
        if direction == 'N':
            ws1['y'] = self.y + 110
            return self.x + 40 if ws1_dir == 'E' else self.x + 110
        if direction == 'S':
            ws1['y'] = self.y + 40
            return self.x + 40 if ws1_dir == 'E' else self.x + 110
        if direction == 'E':
            ws1['x'] = self.x + 40
            return self.y + 40 if ws1_dir == 'S' else self.y + 110
        if direction == 'W':
            ws1['x'] = self.x + 110
            return self.y + 40 if ws1_dir == 'S' else self.y + 110
            
    @staticmethod
    def get_direction_from_theta(theta):
        if theta == 0.0:
            return 'E'
        elif theta == math.pi:
            return 'W'
        elif theta == math.pi/2:
            return 'N'
        elif theta == -math.pi/2:
            return 'S'

    @property
    def remaining_ws_slot(self):
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

