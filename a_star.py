from Map import Map_Obj
import numpy as np


class _SearchNode:
    def __init__(self, state=None):
        self.state = state
        self.g = 0
        self.h = 0
        self.f = 0
        self.status = None  # True = open, False = closed
        self.parent = None
        self.kids: list[_SearchNode] = []

    def calculate_fcost(self):
        self.f = self.g + self.h


class AStar:
    def __init__(self, task, distance="Euclidian", diagonal_movement=True):
        self.map = Map_Obj(task=task)
        self.open_set = {}
        self.closed_set = []
        self.S = {}
        self.distance = distance  # euclidian / manhattan
        self.diagonal_movement = diagonal_movement

    def compute(self) -> "tuple[list, bool]":
        goal = self.map.get_goal_pos()
        start = self.map.get_start_pos()

        # initial node
        n0 = _SearchNode(start)
        n0.g = 0
        n0.h = heuristic(n0.state, goal, self.distance)
        n0.calculate_fcost()
        n0.status = True
        self.open_set[str(n0.state)] = n0
        self.S[str(n0.state)] = n0

        # Compute A*
        while True:
            # No more nodes = failed
            if len(self.open_set) == 0:
                return [], False

            # pop node with lowest f cost
            x: _SearchNode = self.S.get(self.open_set.popitem()[0])
            x.status = False  # Closed/expanded
            self.closed_set.append(str(x.state))

            # Solution?
            if x.state == goal:
                print("solution")
                return x, True

            # Generate successors to parent node (x)
            successors = generate_all_successors(x, self.diagonal_movement)

            # evaluate the successors - i.e. expand child nodes
            for S in successors:
                cost = self.map.get_cell_value(S.state)

                # Valid position/traversable.
                # E.g. value not -1 (out of bound) or 0 (invalid/blocked).
                if cost > 0:

                    # Exist? Fetch node
                    if str(S.state) in self.S:
                        S = self.S.get(str(S.state))
                    x.kids.append(S)

                    # New node (not in open or closed set)
                    if S.status == None:
                        attach_and_eval(S, x, cost, goal, self.distance)
                        S.status = True  # Open
                        self.open_set[str(S.state)] = S.f
                        self.S[str(S.state)] = S
                        # Sort dict -> ascending values
                        # self.open_set = sorted(
                        #     self.open_set.items(), key=lambda x: x[1]
                        # )

                    # Node exist, Cheaper path?
                    elif x.g + cost < S.g:
                        attach_and_eval(S, x, cost, goal, self.distance)
                        if S.status is False:
                            propagate_path_improvements(S, self.map)


def attach_and_eval(child: _SearchNode, parent: _SearchNode, cost, goal, distance):
    """attaches a child node to the node that is now considered its best parent.
    Will comput g (cost to move) and h (estimated cost to goal), and update the f cost for the child.

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


def propagate_path_improvements(parent: _SearchNode, map: Map_Obj):
    """Recurses through the children and updates the nodes with the best parent.
    Ensures that all nodes in the search graph are aware of their current best parent

    Args:
        parent (_SearchNode): _description_
        map (Map_Obj): _description_
    """
    for child in parent.kids:
        cost = map.get_cell_value(child.state)
        if parent.g + cost < child.g:
            child.parent = parent
            child.g = parent.g + cost
            child.calculate_fcost()
            propagate_path_improvements(child, map)


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
        float: the euclidian distance
    """
    return np.linalg.norm(np.asarray(pos) - np.asarray(goal))


def heuristic_manhattan(pos, goal) -> float:
    return NotImplementedError


def get_successors_to_node() -> list:
    return NotImplementedError


def generate_all_successors(
    x: _SearchNode, diagonal_movement: bool
) -> "list[_SearchNode]":
    """will get all the neighboring points to a point/position on the map

    Args:
        x (_SearchNode): a node/point on the map
        diagonal_movement (bool): if diagonal movement is allowed or not

    Returns:
        list[_SearchNode]: the successor nodes
    """
    neighbors = [
        [x.state[0] - 1, x.state[1] + 0],  # Up
        [x.state[0] + 1, x.state[1] + 0],  # down
        [x.state[0] + 0, x.state[1] + 1],  # right
        [x.state[0] + 0, x.state[1] - 1],  # left
    ]
    if diagonal_movement == True:
        diagonal_neighbors = [
            [x.state[0] - 1, x.state[1] - 1],  # top left
            [x.state[0] - 1, x.state[1] + 1],  # top right
            [x.state[0] + 1, x.state[1] - 1],  # bottom left
            [x.state[0] + 1, x.state[1] + 1],  # bottom right
        ]
        neighbors.extend(diagonal_neighbors)
    nodes = []
    for n in neighbors:
        nodes.append(_SearchNode(n))
    return nodes
