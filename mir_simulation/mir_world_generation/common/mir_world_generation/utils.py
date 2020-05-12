
class Utils(object):
    @staticmethod
    def get_connected_nodes(start_cell, walled_edges, max_row, max_col):
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
        neighbours = Utils.get_neighbours(node[0], node[1], max_row, max_col)
        connected_neighbours = [n for n in neighbours \
                                if (node, n) not in walled_edges and (n, node) not in walled_edges]
        return connected_neighbours

    @staticmethod
    def get_neighbours(i, j, max_row, max_col):
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
