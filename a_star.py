"""An implementation of the A star algorithm

Raises:
    ValueError: if the selected metric to calcualte
                distance is not supported, e.g. euclidian / manhattan
"""
import collections
import numpy as np
from Map import Map_Obj


class _SearchNode:
    """This class represent a node/position in the map.
    The state is the exact location [y,x]
    """

    def __init__(self, state=None):
        self.state = state
        self.g = 0
        self.h = 0
        self.f = 0
        self.status = None  # True = open, False = closed
        self.parent = None
        self.kids: list[_SearchNode] = []

    def calculate_fcost(self):
        """Calcualtes the f cost of the node"""
        self.f = self.g + self.h


class AStar:
    """This class, provided a map, will calculate the shortest path to a goal
    using the A star algorithm.
    """

    def __init__(self, task, distance="Euclidian", diagonal_movement=False):
        self.map = Map_Obj(task=task)
        self.solution = None
        self.open_set = {}
        self.closed_set = []
        self.S = {}
        self.distance = distance  # euclidian / manhattan
        self.diagonal_movement = diagonal_movement

    def compute(self):
        """comput the algorithm on the map."""
        goal = self.map.get_goal_pos()
        start = self.map.get_start_pos()

        # initial node
        initial_node = _SearchNode(start)
        initial_node.g = 0
        initial_node.h = heuristic(initial_node.state, goal, self.distance)
        initial_node.calculate_fcost()
        initial_node.status = True
        self.open_set[str(initial_node.state)] = initial_node
        self.S[str(initial_node.state)] = initial_node

        # Compute A*
        while True:
            # No more nodes = failed
            if len(self.open_set) == 0:
                return [], False

            # pop node with lowest f cost
            node: _SearchNode = self.S.get(self.open_set.popitem()[0])
            node.status = False  # Closed/expanded
            self.closed_set.append(str(node.state))

            # Solution?
            if node.state == goal:
                self.solution = node
                return

            # Generate successors to parent node (x)
            successors = generate_all_successors(node, self.diagonal_movement)

            # evaluate the successors - i.e. expand child nodes
            for child_node in successors:
                cost = self.map.get_cell_value(child_node.state)

                # Valid position/traversable.
                # E.g. value not -1 (out of bound) or 0 (invalid/blocked).
                if cost > 0:

                    # Exist? Fetch node
                    if str(child_node.state) in self.S:
                        child_node = self.S.get(str(child_node.state))
                    node.kids.append(child_node)

                    # New node (not in open or closed set)
                    if child_node.status is None:
                        attach_and_eval(child_node, node, cost, goal, self.distance)
                        child_node.status = True  # Open
                        self.open_set[str(child_node.state)] = child_node.f
                        self.S[str(child_node.state)] = child_node
                        # Sort dict -> ascending values
                        self.open_set = collections.OrderedDict(
                            sorted(
                                self.open_set.items(), key=lambda x: x[1], reverse=True
                            )
                        )

                    # Node exist, Cheaper path?
                    elif node.g + cost < child_node.g:
                        attach_and_eval(child_node, node, cost, goal, self.distance)
                        if child_node.status is False:
                            propagate_path_improvements(child_node, self.map)

    def print(self):
        """print the map"""
        if self.solution is None:
            self.map.show_map()
            return

        # get and draw path
        path = get_path(self.solution)
        for point in path:
            self.map.set_cell_value(point, " ")

        # Repaint goal/start position
        self.map.set_cell_value(self.map.get_goal_pos(), " G ")
        self.map.set_cell_value(self.map.get_start_pos(), " S ")

        # print map
        self.map.show_map()


def attach_and_eval(child: _SearchNode, parent: _SearchNode, cost, goal, distance):
    """attaches a child node to the node that is now considered its best parent.
    Will comput g (cost to move) and h (estimated cost to goal),
    and update the f cost for the child.

    Args:
        child (_SearchNode): the child node/neighbor
        parent (_SearchNode): the parent node
        cost (float): the cost of moving from parent to child
        goal (list): the goal position
        distance (bool): metric used to calculate distance.
    """
    child.parent = parent
    child.g = parent.g + cost
    child.h = heuristic(child.state, goal, distance)
    child.calculate_fcost()


def propagate_path_improvements(parent: _SearchNode, the_map: Map_Obj):
    """Recurses through the children and updates the nodes with the best parent.
    Ensures that all nodes in the search graph are aware of their current best parent

    Args:
        parent (_SearchNode): _description_
        map (Map_Obj): _description_
    """
    for child in parent.kids:
        cost = the_map.get_cell_value(child.state)
        if parent.g + cost < child.g:
            child.parent = parent
            child.g = parent.g + cost
            child.calculate_fcost()
            propagate_path_improvements(child, the_map)


def heuristic(pos, goal, distance) -> float:
    """Calulate the heuristic (distance to goal)

    Args:
        pos (list): the current position [y,x]
        goal (list): the position of the goal [y,x]
        distance (str): metric used to calculate distance

    Raises:
        ValueError: metric does not exist

    Returns:
        float: h(x) -> distance to goal as a float value
    """
    supported_distances = ["euclidian", "manhattan"]
    distance = distance.lower()
    if distance in supported_distances:
        if distance == supported_distances[0]:
            return heuristic_euclidian(pos, goal)
        if distance == supported_distances[1]:
            return heuristic_manhattan(pos, goal)
    else:
        raise ValueError(
            f"{distance}, is not supported, please use {supported_distances}"
        )


def heuristic_euclidian(pos, goal) -> float:
    """Calculates the euclidian distance between two points

    Args:
        pos (list): a point as a list [y,x]
        goal (list): the goal [y,x]

    Returns:
        float: the euclidian distance from point to goal
    """
    return np.linalg.norm(np.asarray(pos) - np.asarray(goal))


def heuristic_manhattan(pos, goal) -> float:
    """Calculates the manhattan distance between two points

    Args:
        pos (list): a point in the map [y,x]
        goal (list): the goal [y,x]

    Returns:
        float: the manhattan distance from point to goal
    """
    return sum(abs(val1 - val2) for val1, val2 in zip(pos, goal))


def generate_all_successors(
    node: _SearchNode, diagonal_movement: bool
) -> "list[_SearchNode]":
    """will get all the neighboring points to a point/position on the map

    Args:
        x (_SearchNode): a node/point on the map
        diagonal_movement (bool): if diagonal movement is allowed or not

    Returns:
        list[_SearchNode]: the successor nodes
    """
    neighbors = [
        [node.state[0] - 1, node.state[1] + 0],  # Up
        [node.state[0] + 1, node.state[1] + 0],  # down
        [node.state[0] + 0, node.state[1] + 1],  # right
        [node.state[0] + 0, node.state[1] - 1],  # left
    ]
    if diagonal_movement is True:
        diagonal_neighbors = [
            [node.state[0] - 1, node.state[1] - 1],  # top left
            [node.state[0] - 1, node.state[1] + 1],  # top right
            [node.state[0] + 1, node.state[1] - 1],  # bottom left
            [node.state[0] + 1, node.state[1] + 1],  # bottom right
        ]
        neighbors.extend(diagonal_neighbors)
    nodes = []
    for point in neighbors:
        nodes.append(_SearchNode(point))
    return nodes


def get_path(node: _SearchNode) -> "list[list]":
    """Will get all the points to the path

    Args:
        node (_SearchNode): the solution node

    Returns:
        list[list]: a list of all the points of the path
    """
    path = []

    # find all the parents, i.e. the path to goal position
    while node.parent is not None:
        path.append(node.state)
        node = node.parent
    return path
