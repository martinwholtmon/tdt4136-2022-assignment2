from Map import Map_Obj
import numpy as np


class _SearchNode:
    def __init__(self, pos) -> None:
        self.state = pos
        self.g = 0
        self.h = 0
        self.f = 0
        self.status = True  # open
        self.parent = _SearchNode
        self.kids = []

    def calculate_fcost(self):
        self.f = self.g + self.h


class AStar:
    def __init__(self, task, distance="Euclidian", diagonal_movement=True) -> None:
        self.map = Map_Obj(task=task)
        self.open_set = []
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
        self.open_set.append(n0)

        while True:
            # No more nodes = failed
            if len(self.open_set) == 0:
                return [], False

            # Get node with lowest f cost
            x = self.open_set.pop()
            self.closed_set.append(x)

            # Solution?
            if x.state == goal:
                return x, True

            # Find neighbor points
            neighbors = get_neighboring_points(x, self.diagonal_movement)

            # Generate successors to node
            for neighbor in neighbors:
                # Valid position/traversable.
                # E.g. value not -1 (out of bound) or 0 (invalid/blocked).
                if self.map.get_cell_value(neighbor) > 0:
                    # Exist?
                    if str(neighbor) not in self.S:
                        print("yda")


def attach_and_eval():
    return NotImplementedError


def propagate_path_improvements():
    return NotImplementedError


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


def get_neighboring_points(x, diagonal_movement) -> "list[list]":
    """will get all the neighboring points to a point/position on the map

    Args:
        x (_SearchNode): a node/point on the map
        diagonal_movement (bool): if diagonal movement is allowed or not
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
    return neighbors
