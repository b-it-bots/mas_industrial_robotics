
class Utils(object):
    @staticmethod
    def get_connected_nodes(start_cell, walled_edges, max_row, max_col):
        """
        Find the connected graph starting from `start_cell` in a grid of size 
        `max_row` x `max_col`. All the edges except `walled_edges` are considered
        as connected.

        :start_cell: tuple (int, int)
        :walled_edges: list of list of tuple (int, int)
        :max_row: int
        :max_col: int
        :returns: list of tuple (int, int)

        """
        connected_nodes = []
        fringe = [start_cell]
        while len(fringe) > 0:
            current = fringe.pop(0)
            if current in connected_nodes:
                continue
            connected_neighbours = Utils.get_connected_neighbour(current, walled_edges, max_row, max_col)
            connected_nodes.append(current)
            for neighbour in connected_neighbours:
                if neighbour not in connected_nodes:
                    fringe.append(neighbour)
        return connected_nodes
 
    @staticmethod
    def get_connected_neighbour(node, walled_edges, max_row, max_col):
        """
        Return neighbours of a cell (Node object) which are connected (not walled off).
        
        :node: tuple (int, int)
        :walled_edges: list of list of tuple (int, int)
        :max_row: int
        :max_col: int
        :returns: list of tuple (int, int)

        """
        neighbours = Utils.get_neighbours(node[0], node[1], max_row, max_col)
        connected_neighbours = [n for n in neighbours \
                                if (node, n) not in walled_edges and (n, node) not in walled_edges]
        return connected_neighbours

    @staticmethod
    def get_neighbours(i, j, max_row, max_col):
        """
        Return all possible neighbour of a cell at position (`i`, `j`) in a grid
        of size `max_row` x `max_col`.

        :i: int
        :j: int
        :max_row: int
        :max_col: int
        :returns: list of tuple (int, int)

        """
        neighbours = []
        if i != 0:
            neighbours.append((i-1, j))
        if i != max_row-1:
            neighbours.append((i+1, j))
        if j != 0:
            neighbours.append((i, j-1))
        if j != max_col-1:
            neighbours.append((i, j+1))
        return neighbours
