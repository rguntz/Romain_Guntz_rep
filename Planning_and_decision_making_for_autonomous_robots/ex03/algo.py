from dataclasses import dataclass
from enum import Enum, unique
from typing import Mapping, Optional, Any
import math
import warnings
from typing import TypeVar, List
import osmnx as ox


warnings.filterwarnings("ignore", category=FutureWarning, message=".*euclidean_dist_vec.*")


infinity = math.inf


from networkx import MultiDiGraph

from pdm4ar.exercises.ex02.structures import AdjacencyList, X


class EdgeNotFound(Exception):
    pass


class NodePropertyNotFound(Exception):
    pass


X = TypeVar("X")
NODE_NAME = TypeVar("NODE_NAME")


class Node_identity:
    def __init__(self, name: NODE_NAME):
        self.name = name
        self.parent = None  # Initialize parent to None

    def set_parent(self, parent: NODE_NAME):
        self.parent = parent  # Set the parent to the provided Person instance


class NodeOriginFinder:
    def origin_finder(self, solution_tot_with_affiliation: List[Node_identity], s: X) -> List[X]:

        for i in range(len(solution_tot_with_affiliation)):
            if solution_tot_with_affiliation[i].name == s:
                last_element = solution_tot_with_affiliation[i]

        Familiation = [last_element.name]

        while last_element.parent != None:
            Familiation.insert(0, last_element.parent)
            for i in range(len(solution_tot_with_affiliation)):
                if solution_tot_with_affiliation[i].name == last_element.parent:
                    last_element = solution_tot_with_affiliation[i]

        return Familiation


@unique
class NodeAttribute(str, Enum):
    LONGITUDE = "x"
    LATITUDE = "y"


@unique
class TravelSpeed(float, Enum):
    HIGHWAY = 100.0 / 3.6
    SECONDARY = 70.0 / 3.6
    CITY = 50.0 / 3.6
    PEDESTRIAN = 5.0 / 3.6


@dataclass
class WeightedGraph:
    adj_list: AdjacencyList
    weights: Mapping[tuple[X, X], float]
    _G: MultiDiGraph

    def get_weight(self, u: X, v: X) -> Optional[float]:
        """
        :param u: The "from" of the edge
        :param v: The "to" of the edge
        :return: The weight associated to the edge, raises an Exception if the edge does not exist
        """
        try:
            return self.weights[(u, v)]
        except KeyError:
            raise EdgeNotFound(f"Cannot find weight for edge: {(u, v)}")

    def _get_node_attribute(self, node_id: X, attribute: NodeAttribute) -> Any:
        """
        Private method of class WeightedGraph
        :param node_id: The node id
        :param attribute: The node attribute name
        :return: The corresponding value
        """
        return self._G.nodes[node_id][attribute]

    def get_node_coordinates(self, u: X) -> tuple[float, float]:
        """
        Method of class WeightedGraph:
        :param u: node id
        :return (x, y): coordinates (LON & LAT) of node u
        """
        return (
            self._G.nodes[u][NodeAttribute.LONGITUDE],
            self._G.nodes[u][NodeAttribute.LATITUDE],
        )


from abc import ABC, abstractmethod
from dataclasses import dataclass
import heapq  # you may find this helpful

from osmnx.distance import great_circle_vec

from pdm4ar.exercises.ex02.structures import X, Path
from pdm4ar.exercises.ex03.structures import WeightedGraph, TravelSpeed


@dataclass
class InformedGraphSearch(ABC):
    graph: WeightedGraph

    @abstractmethod
    def path(self, start: X, goal: X) -> Path:
        # Abstract function. Nothing to do here.
        pass


@dataclass
class UniformCostSearch(InformedGraphSearch):
    def path(self, start: X, goal: X) -> Path:
        # todo

        self.start = start
        self.goal = goal

        s0 = Node_identity(self.start)
        Q: List[X] = []  # Queue for exploration
        V_with_affiliation: List[Node_identity] = []
        V_with_affiliation.append(s0)
        Q.append(self.start)

        # graph_adjesant_list = self.graph.adj_list
        cost_to_reach = {key: float("inf") for key in self.graph.adj_list}

        cost_to_reach[self.start] = 0

        while Q:

            Q = sorted(Q, key=lambda ID: cost_to_reach[ID])
            s = Q[0]
            Q.pop(0)

            if s == self.goal:
                ## Added :
                NodeOriginFinder_object = NodeOriginFinder()
                solution = NodeOriginFinder_object.origin_finder(V_with_affiliation, s)
                return solution

            else:

                adjacent_to_node = self.graph.adj_list[s]

                for node in adjacent_to_node:

                    new_cost_to_reach = self.graph.get_weight(s, node) + cost_to_reach[s]

                    if new_cost_to_reach < cost_to_reach[node]:
                        cost_to_reach[node] = new_cost_to_reach

                        V_adding = Node_identity(node)
                        V_adding.set_parent(s)

                        V_with_affiliation = [v for v in V_with_affiliation if v.name != V_adding.name]
                        V_with_affiliation.append(V_adding)

                        if node not in Q:
                            Q.append(node)

        return []


@dataclass
class Astar(InformedGraphSearch):

    # Keep track of how many times the heuristic is called
    heuristic_counter: int = 0
    # Allows the tester to switch between calling the students heuristic function and
    # the trivial heuristic (which always returns 0). This is a useful comparison to
    # judge how well your heuristic performs.
    use_trivial_heuristic: bool = False  # Changed here

    def heuristic(self, u: X, v: X) -> float:
        # Increment this counter every time the heuristic is called, to judge the performance
        # of the algorithm
        self.heuristic_counter += 1
        if self.use_trivial_heuristic:
            return 0
        else:
            # return the heuristic that the student implements
            return self._INTERNAL_heuristic(u, v)

    # Implement the following two functions

    def _INTERNAL_heuristic(self, u: X, v: X) -> float:
        # Implement your heuristic here. Your `path` function should NOT call
        # this function directly. Rather, it should call `heuristic`
        # todo

        lat1, lon1 = self.graph.get_node_coordinates(u)
        lat2, lon2 = self.graph.get_node_coordinates(v)

        distance_u_v = ox.distance.great_circle_vec(lat1, lon1, lat2, lon2)
        time = distance_u_v / (TravelSpeed.HIGHWAY)  # Changed here for commit

        return time

    def path(self, start: X, goal: X) -> Path:
        # todo

        self.start = start
        self.goal = goal

        s0 = Node_identity(self.start)
        Q: List[X] = []  # Queue for exploration
        V_with_affiliation: List[Node_identity] = []
        V_with_affiliation.append(s0)
        Q.append(self.start)

        # graph_adjesant_list = self.graph.adj_list
        cost_to_reach = {key: float("inf") for key in self.graph.adj_list}

        cost_to_reach[self.start] = 0

        while Q:

            # Change little think to commit

            heuristic_values = [self.heuristic(ID, self.goal) for ID in Q]
            # Step 2: Create a combined array of (ID, total_cost)
            combined_costs = [(ID, cost_to_reach[ID] + heuristic) for ID, heuristic in zip(Q, heuristic_values)]
            # Step 3: Sort based on the total cost
            Q_sorted = sorted(combined_costs, key=lambda x: x[1])
            # Step 4: Extract sorted IDs from the sorted combined array
            Q = [ID for ID, _ in Q_sorted]

            s = Q[0]
            Q.pop(0)

            if s == self.goal:  # changed here
                ## Added :
                NodeOriginFinder_object = NodeOriginFinder()
                solution = NodeOriginFinder_object.origin_finder(V_with_affiliation, s)
                return solution

            else:

                adjacent_to_node = self.graph.adj_list[s]

                for node in adjacent_to_node:
                    new_cost_to_reach = self.graph.get_weight(s, node) + cost_to_reach[s]

                    if new_cost_to_reach < cost_to_reach[node]:
                        cost_to_reach[node] = new_cost_to_reach

                        V_adding = Node_identity(node)
                        V_adding.set_parent(s)

                        V_with_affiliation = [v for v in V_with_affiliation if v.name != V_adding.name]
                        V_with_affiliation.append(V_adding)

                        Q.append(node)

        return []


def compute_path_cost(wG: WeightedGraph, path: Path):
    """A utility function to compute the cumulative cost along a path"""
    if not path:
        return float("inf")
    total: float = 0
    for i in range(1, len(path)):
        inc = wG.get_weight(path[i - 1], path[i])
        total += inc
    return total
