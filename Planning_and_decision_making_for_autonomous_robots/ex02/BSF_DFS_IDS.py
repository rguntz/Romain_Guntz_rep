from abc import abstractmethod, ABC

from pdm4ar.exercises.ex02.structures import AdjacencyList, X, Path, OpenedNodes


from typing import TypeVar, Sequence, List

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


class NodeOriginFinder_iterative:
    def origin_finder_iterative(self, solution_tot_with_affiliation: List[Node_identity], s: X) -> X:

        for i in range(len(solution_tot_with_affiliation)):
            if solution_tot_with_affiliation[i].name == s:
                last_element = solution_tot_with_affiliation[i]

        Familiation = [last_element.name]

        while last_element.parent != None:
            Familiation.insert(0, last_element.parent)
            for i in range(len(solution_tot_with_affiliation)):
                if solution_tot_with_affiliation[i].name == last_element.parent:
                    last_element = solution_tot_with_affiliation[i]

        return len(Familiation)


class NodeOriginFinder_2:
    def origin_finder_2(self, solution_tot_with_affiliation: List[Node_identity], s: X) -> List[X]:

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


class GraphSearch(ABC):
    @abstractmethod
    def search(self, graph: AdjacencyList, start: X, goal: X) -> tuple[Path, OpenedNodes]:
        """
        :param graph: The given graph as an adjacency list
        :param start: The initial state (i.e. a node)
        :param goal: The goal state (i.e. a node)
        :return: The path from start to goal as a Sequence of states, None if a path does not exist
        """
        pass


class DepthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> tuple[Path, OpenedNodes]:
        self.start = start
        self.goal = goal

        Q: List[X] = []  # Queue for exploration
        V: List[X] = []  # Visited nodes
        visited: List[X] = []

        s0 = Node_identity(self.start)
        Q.append(self.start)
        V.append(s0.name)

        V_with_affiliation: List[Node_identity] = []
        V_with_affiliation.append(s0)

        while Q:

            Q_iteration: List[X] = []
            s = Q[0]
            Q.pop(0)
            visited.append(s)

            if s == self.goal:
                ## Added :

                NodeOriginFinder_object = NodeOriginFinder()
                solution = NodeOriginFinder_object.origin_finder(V_with_affiliation, s)
                return solution, visited

            else:

                adjacent_to_node = graph[s]

                for node in adjacent_to_node:
                    if node not in V:

                        V_adding = Node_identity(node)
                        V_adding.set_parent(s)

                        V.append(V_adding.name)
                        V_with_affiliation.append(V_adding)
                        Q_iteration.append(node)

                Q_iteration.sort()
                Q = Q_iteration + Q

        return [], visited


class BreadthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> tuple[Path, OpenedNodes]:
        self.start = start
        self.goal = goal

        Q: List[X] = []  # Queue for exploration
        V: List[X] = []  # Visited nodes
        visited: List[X] = []

        s0 = Node_identity(self.start)
        Q.append(self.start)
        V.append(s0.name)

        V_with_affiliation: List[Node_identity] = []
        V_with_affiliation.append(s0)

        while Q:
            s = Q[0]
            visited.append(s)

            if s == self.goal:

                ## Added :

                NodeOriginFinder_object = NodeOriginFinder_2()
                solution = NodeOriginFinder_object.origin_finder_2(V_with_affiliation, s)

                return solution, visited

            else:
                Q.pop(0)
                adjacent_to_node = graph[s]
                adjacent_to_node = list(adjacent_to_node)
                adjacent_to_node.sort()

                for node in adjacent_to_node:
                    if node not in V:

                        V_adding = Node_identity(node)
                        V_adding.set_parent(s)

                        V.append(node)
                        V_with_affiliation.append(V_adding)

                        Q.append(node)  # Add nodes to the front to maintain DFS order

        return [], visited


class IterativeDeepening(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> tuple[Path, OpenedNodes]:
        self.start = start
        self.goal = goal
        max_depth = len(graph) - 2

        for depth in range(1, max_depth):

            Q: List[X] = []  # Queue for exploration
            V: List[X] = []  # Visited nodes
            visited: List[X] = []

            s0 = Node_identity(self.start)
            Q.append(self.start)
            V.append(s0.name)

            V_with_affiliation: List[Node_identity] = []
            V_with_affiliation.append(s0)

            while Q:
                Q_iteration: List[X] = []
                s = Q[0]
                visited.append(s)

                if s == self.goal:

                    ## Added :

                    NodeOriginFinder_object = NodeOriginFinder_2()
                    solution = NodeOriginFinder_object.origin_finder_2(V_with_affiliation, s)

                    return solution, visited

                NodeOriginFinder_iterative_object = NodeOriginFinder_iterative()
                depth_s = NodeOriginFinder_iterative_object.origin_finder_iterative(V_with_affiliation, s)

                if s != self.goal and depth_s < (depth + 1):

                    Q.pop(0)
                    adjacent_to_node = graph[s]
                    adjacent_to_node = list(adjacent_to_node)
                    adjacent_to_node.sort()

                    for node in adjacent_to_node:
                        if node not in V:

                            V_adding = Node_identity(node)
                            V_adding.set_parent(s)

                            V.append(V_adding.name)
                            V_with_affiliation.append(V_adding)
                            Q_iteration.append(node)

                    Q = Q_iteration + Q

                else:  ## Change to test

                    Q.pop(0)

        return [], visited
