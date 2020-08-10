import math
import random


class Node(object):

    """
    An object representation of a cell in a grid of size ``row`` x ``col``. Each cell
    is a square of dimension ``size`` (in cm) which can hold **atmost 2** workstations.
    """

    ws_length = 80  # cm
    ws_width = 50  # cm

    def __init__(self, col=0, row=0, size=150):
        self.size = size
        self.x = col * self.size
        self.y = row * self.size
        self.ws = []
        # North, South, East, West (north will be positive Y axis)
        self.probable_ws_direction = ["N", "S", "E", "W"]

    def add_ws(self):
        """
        Add a workstation at random in the cell.
        If a workstation is already present then move that one in a way such that
        there is no overlap between the two
        """
        min_len_pos = self.ws_length / 2
        max_len_pos = self.size - (self.ws_length / 2)
        min_wid_pos = self.ws_width / 2
        max_wid_pos = self.size - (self.ws_width / 2)

        direction = random.choice(self.probable_ws_direction)
        self.probable_ws_direction.remove(direction)
        if direction == "N":
            if "S" in self.probable_ws_direction:
                self.probable_ws_direction.remove("S")
            x = (
                self._resolve_overlap(direction)
                if len(self.ws) > 0
                else random.randint(self.x + min_len_pos, self.x + max_len_pos)
            )
            y = self.y + min_wid_pos
            theta = math.pi / 2
        if direction == "S":
            if "N" in self.probable_ws_direction:
                self.probable_ws_direction.remove("N")
            x = (
                self._resolve_overlap(direction)
                if len(self.ws) > 0
                else random.randint(self.x + min_len_pos, self.x + max_len_pos)
            )
            y = self.y + max_wid_pos
            theta = -math.pi / 2
        if direction == "E":
            if "W" in self.probable_ws_direction:
                self.probable_ws_direction.remove("W")
            x = self.x + max_wid_pos
            y = (
                self._resolve_overlap(direction)
                if len(self.ws) > 0
                else random.randint(self.y + min_len_pos, self.y + max_len_pos)
            )
            theta = 0.0
        if direction == "W":
            if "E" in self.probable_ws_direction:
                self.probable_ws_direction.remove("E")
            x = self.x + min_wid_pos
            y = (
                self._resolve_overlap(direction)
                if len(self.ws) > 0
                else random.randint(self.y + min_len_pos, self.y + max_len_pos)
            )
            theta = math.pi

        self.ws.append({"x": x, "y": y, "theta": theta})

    def _resolve_overlap(self, direction):
        """
        Given the direction of a workstation that needs to be added to the cell,
        resolve any possible overlap between it and the already present WS.
        The already present WS is moved to a corner and the new WS will be spawned
        to diagonally opposite corner.

        :direction: str ('N', 'S', 'E', or 'W')
        :returns: int

        """
        min_len_pos = self.ws_length / 2
        max_len_pos = self.size - (self.ws_length / 2)
        min_wid_pos = self.ws_width / 2
        max_wid_pos = self.size - (self.ws_width / 2)

        ws1 = self.ws[0]  # already present WS
        ws1_dir = Node.get_direction_from_theta(ws1["theta"])
        if direction == "N":
            ws1["y"] = self.y + max_len_pos
            return self.x + min_len_pos if ws1_dir == "E" else self.x + max_len_pos
        if direction == "S":
            ws1["y"] = self.y + min_len_pos
            return self.x + min_len_pos if ws1_dir == "E" else self.x + max_len_pos
        if direction == "E":
            ws1["x"] = self.x + min_len_pos
            return self.y + min_len_pos if ws1_dir == "S" else self.y + max_len_pos
        if direction == "W":
            ws1["x"] = self.x + max_len_pos
            return self.y + min_len_pos if ws1_dir == "S" else self.y + max_len_pos

    @staticmethod
    def get_direction_from_theta(theta):
        """
        Return a direction ('N', 'S', 'E', 'W') from theta (-pi to pi)

        :theta: float
        :returns: str

        """
        if theta == 0.0:
            return "E"
        elif theta == math.pi:
            return "W"
        elif theta == math.pi / 2:
            return "N"
        elif theta == -math.pi / 2:
            return "S"

    @property
    def remaining_ws_slot(self):
        """
        Calculate how many more workstation can be spawned into this cell

        :returns: int

        """
        remaining_ws_slot = 0
        if "N" in self.probable_ws_direction or "S" in self.probable_ws_direction:
            remaining_ws_slot += 1
        if "E" in self.probable_ws_direction or "W" in self.probable_ws_direction:
            remaining_ws_slot += 1
        return remaining_ws_slot

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        string = "<"
        string += "x: " + str(self.x) + ", "
        string += "y: " + str(self.y) + ", "
        string += "prob: " + str(self.probable_ws_direction) + ", "
        string += "ws: " + str(self.ws)
        string += ">"
        return string
