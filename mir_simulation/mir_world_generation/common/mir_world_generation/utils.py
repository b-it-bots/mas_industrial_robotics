class Utils(object):
    @staticmethod
    def get_connected_nodes(start_cell, walled_edges, max_row, max_col):
        """
        Find the connected graph starting from ``start_cell`` in a grid of size
        ``max_row`` x ``max_col``. All the edges except ``walled_edges`` are considered
        as connected.

        :param start_cell: starting position of the robot in grid
        :type start_cell: tuple (int, int)
        :param walled_edges: edges in a graph which are not connected
        :type walled_edges: list (list (tuple (int, int)))
        :param max_row: number of rows in the grid
        :type max_row: int
        :param max_col: number of columns in the grid
        :type max_col: int
        :return: connected graph of cells
        :rtype: list (tuple(int, int))

        """
        connected_nodes = []
        fringe = [start_cell]
        while len(fringe) > 0:
            current = fringe.pop(0)
            if current in connected_nodes:
                continue
            connected_neighbours = Utils.get_connected_neighbour(
                current, walled_edges, max_row, max_col
            )
            connected_nodes.append(current)
            for neighbour in connected_neighbours:
                if neighbour not in connected_nodes:
                    fringe.append(neighbour)
        return connected_nodes

    @staticmethod
    def get_connected_neighbour(node, walled_edges, max_row, max_col):
        """
        Return neighbours of a cell (represented by :class:`mir_world_generation.node.Node`)
        which are connected (not walled off).

        :param node: cell/node in a graph made from a grid
        :type node: tuple (int, int)
        :param walled_edges: edges in a graph which are not connected
        :type walled_edges: list (list (tuple (int, int)))
        :param max_row: number of rows in the grid
        :type max_row: int
        :param max_col: number of columns in the grid
        :type max_col: int
        :return: neighbours of a cell which are connected (not walled off)
        :rtype: list (tuple (int, int))

        """
        neighbours = Utils.get_neighbours(node[0], node[1], max_row, max_col)
        connected_neighbours = [
            n
            for n in neighbours
            if (node, n) not in walled_edges and (n, node) not in walled_edges
        ]
        return connected_neighbours

    @staticmethod
    def get_neighbours(i, j, max_row, max_col):
        """
        Return all possible neighbour of a cell at position (``i``, ``j``) in a grid
        of size ``max_row`` x ``max_col``.

        :param i: row index
        :type i: int
        :param j: column index
        :type j: int
        :param max_row: number of rows in the grid
        :type max_row: int
        :param max_col: number of columns in the grid
        :type max_col: int
        :return: all possible neighbours
        :rtype: list (tuple (int, int))

        """
        neighbours = []
        if i != 0:
            neighbours.append((i - 1, j))
        if i != max_row - 1:
            neighbours.append((i + 1, j))
        if j != 0:
            neighbours.append((i, j - 1))
        if j != max_col - 1:
            neighbours.append((i, j + 1))
        return neighbours
