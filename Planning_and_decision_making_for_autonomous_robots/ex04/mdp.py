from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray
from pdm4ar.exercises.ex04.structures import Action, Policy, State, ValueFunc
from enum import IntEnum, unique
import os


@unique
class Cell(IntEnum):
    GOAL = 0
    START = 1
    GRASS = 2
    SWAMP = 3
    WORMHOLE = 4
    CLIFF = 5


class GridMdp:
    def __init__(self, grid: NDArray[np.int64], gamma: float = 0.9):
        assert len(grid.shape) == 2, "Map is invalid"
        self.grid = grid
        """The map"""
        self.gamma: float = gamma
        """Discount factor"""

    def get_actions_to_do(self, state: State):
        # This function only describe the four different moves of the robot without any
        # assumption on the type of movement avaiable for the type of movement possible
        grid = np.array(self.grid)
        row = state[0]
        column = state[1]

        if grid[row][column] != Cell.GOAL:
            actions_set = {Action.NORTH, Action.WEST, Action.SOUTH, Action.EAST, Action.ABANDON}

            if (row - 1) < 0:
                actions_set.remove(Action.NORTH)
            if row + 1 >= grid.shape[0]:
                actions_set.remove(Action.SOUTH)
            if (column - 1) < 0:
                actions_set.remove(Action.WEST)
            if column + 1 >= grid.shape[1]:
                actions_set.remove(Action.EAST)

            if (row - 1) >= 0 and grid[row - 1][column] == Cell.CLIFF:
                actions_set.remove(Action.NORTH)
            if (row + 1 < grid.shape[0]) and grid[row + 1][column] == Cell.CLIFF:
                actions_set.remove(Action.SOUTH)
            if (column - 1) >= 0 and grid[row][column - 1] == Cell.CLIFF:
                actions_set.remove(Action.WEST)
            if (column + 1 < grid.shape[1]) and grid[row][column + 1] == Cell.CLIFF:
                actions_set.remove(Action.EAST)

        elif grid[row][column] == Cell.GOAL:
            actions_set = {Action.STAY}
        return actions_set

    def get_wormholes(self):
        grid = self.grid
        grid = np.array(grid)
        coordinates = np.argwhere(grid == 4)
        coordinates = set(map(tuple, coordinates))
        return coordinates, len(coordinates)

    def get_start(self):
        grid = self.grid
        grid = np.array(grid)
        coordinates = np.argwhere(grid == 1)
        coordinates = coordinates[0]
        return (coordinates[0], coordinates[1])

    def get_possible_moves_even_non_admissible(self, state: State):
        grid = np.array(self.grid)
        actions_set = {Action.NORTH, Action.WEST, Action.SOUTH, Action.EAST}
        coordinates_set = set()  # Initialize an empty set for coordinates
        row = state[0]
        column = state[1]

        for action in actions_set:
            if action == Action.NORTH:
                if (row - 1) < 0 or grid[row - 1][column] == Cell.CLIFF:
                    coordinates_set.add(self.get_start())  # Use add() instead of append()
                else:
                    coordinates_set.add((row - 1, column))
            elif action == Action.WEST:
                if (column - 1) < 0 or grid[row][column - 1] == Cell.CLIFF:
                    coordinates_set.add(self.get_start())
                else:
                    coordinates_set.add((row, column - 1))
            elif action == Action.SOUTH:
                if (row + 1) >= grid.shape[0] or grid[row + 1][column] == Cell.CLIFF:  # Fixed condition
                    coordinates_set.add(self.get_start())
                else:
                    coordinates_set.add((row + 1, column))
            elif action == Action.EAST:
                if (column + 1) >= grid.shape[1] or grid[row][column + 1] == Cell.CLIFF:  # Fixed condition
                    coordinates_set.add(self.get_start())
                else:
                    coordinates_set.add((row, column + 1))

            coordinates_set_with_wormholes = coordinates_set
            for coordinate in coordinates_set:
                if grid[coordinate[0], coordinate[1]] == Cell.WORMHOLE:
                    wormhole_coordinates, _ = self.get_wormholes()
                    coordinates_set_with_wormholes = coordinates_set.union(wormhole_coordinates)
                    break

        if grid[state[0]][state[1]] == Cell.SWAMP:
            coordinates_set_with_wormholes.add(self.get_start())

        return coordinates_set_with_wormholes  # Return the unique coordinates set directly

    def find_number_of_adjacant_wormholes(self, state: State):
        grid = np.array(self.grid)
        row = state[0]
        col = state[1]
        number_of_adjacant_wormholes = 0

        if (row - 1) >= 0:
            if grid[row - 1][col] == Cell.WORMHOLE:
                number_of_adjacant_wormholes = number_of_adjacant_wormholes + 1
        if row + 1 < grid.shape[0]:
            if grid[row + 1][col] == Cell.WORMHOLE:
                number_of_adjacant_wormholes = number_of_adjacant_wormholes + 1
        if (col - 1) >= 0:
            if grid[row][col - 1] == Cell.WORMHOLE:
                number_of_adjacant_wormholes = number_of_adjacant_wormholes + 1
        if col + 1 < grid.shape[1]:
            if grid[row][col + 1] == Cell.WORMHOLE:
                number_of_adjacant_wormholes = number_of_adjacant_wormholes + 1

        return number_of_adjacant_wormholes

    def number_outside_cases_or_cliff(self, state: State):
        grid = np.array(self.grid)
        row = state[0]
        col = state[1]

        numberoutside_cases_or_cliff = 0

        if row == 0:
            numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1
        if row == grid.shape[0] - 1:
            numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1
        if col == 0:
            numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1
        if col == grid.shape[1] - 1:
            numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        if row - 1 >= 0:
            if grid[row - 1][col] == Cell.CLIFF:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        if row + 1 < grid.shape[0]:
            if grid[row + 1][col] == Cell.CLIFF:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        if col - 1 >= 0:
            if grid[row][col - 1] == Cell.CLIFF:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1
        if col + 1 < grid.shape[1]:
            if grid[row][col + 1] == Cell.CLIFF:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        return numberoutside_cases_or_cliff

    def number_access_to_start(self, state: State):
        grid = np.array(self.grid)
        row = state[0]
        col = state[1]

        numberoutside_cases_or_cliff = 0

        if row - 1 >= 0:
            if grid[row - 1][col] == Cell.START:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        if row + 1 < grid.shape[0]:
            if grid[row + 1][col] == Cell.START:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        if col - 1 >= 0:
            if grid[row][col - 1] == Cell.START:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1
        if col + 1 < grid.shape[1]:
            if grid[row][col + 1] == Cell.START:
                numberoutside_cases_or_cliff = numberoutside_cases_or_cliff + 1

        return numberoutside_cases_or_cliff

    def motion_for_Grass_and_wormholes(self, state: State, action: Action, next_state: State):
        grid = np.array(self.grid)
        all_possible_outcomes = self.get_possible_moves_even_non_admissible((state[0], state[1]))
        transitions_probability = [(next_state, None, None) for next_state in all_possible_outcomes]

        row = state[0]
        col = state[1]

        # Initialize new_row and new_col
        new_row, new_col = row, col

        if action == Action.NORTH:
            new_row = row - 1
        elif action == Action.SOUTH:
            new_row = row + 1
        elif action == Action.WEST:
            new_col = col - 1
        elif action == Action.EAST:
            new_col = col + 1

        if action == Action.ABANDON:
            return (self.get_start(), 1, None)

        elif action != Action.ABANDON:

            ## First, do the case of an adjacant wormhole
            proba = 0.75
            proba_remaining = (1 - proba) / 3
            wormholes, number_of_wormholes = self.get_wormholes()
            n_adjacent_wormholes = self.find_number_of_adjacant_wormholes(state)

            if (new_row, new_col) in wormholes:
                probability_wormhole = (1 / number_of_wormholes) * (
                    proba + (n_adjacent_wormholes - 1) * proba_remaining
                )

                for wormholes_to_update in wormholes:
                    transitions_probability = [
                        (
                            current_next_state,
                            probability_wormhole if current_next_state == wormholes_to_update else original_prob,
                            None,
                        )
                        for current_next_state, original_prob, _ in transitions_probability
                    ]

            # If the first movement is on the start case :

            elif grid[new_row][new_col] == Cell.START:
                transitions_probability = [
                    (next_state, proba, None) if next_state == self.get_start() else (next_state, original_prob, None)
                    for next_state, original_prob, _ in transitions_probability
                ]

            else:
                ## If the next move is not a wormhole
                transitions_probability = [
                    (next_state, proba, None) if next_state == (new_row, new_col) else (next_state, original_prob, None)
                    for next_state, original_prob, _ in transitions_probability
                ]

            ## probability to reach the start again which is equal to number of cliffs or edges
            if grid[new_row][new_col] != Cell.START:
                number_outside_cases_or_cliff_ = self.number_outside_cases_or_cliff(state)
                number_access_to_start_ = self.number_access_to_start(state)
                proba_start = (number_outside_cases_or_cliff_ + number_access_to_start_) * proba_remaining
                # Dont forget to add the case where it when directly to the start (0.75 chance to go start)
                transitions_probability = [
                    (
                        (next_state, (proba_start + original_prob if original_prob is not None else proba_start), None)
                        if next_state == self.get_start()
                        else (next_state, original_prob if original_prob is not None else 0.0, None)
                    )
                    for next_state, original_prob, _ in transitions_probability
                ]

            ## Update if no wormhole in the next move but worlhole in the neighborhood
            if n_adjacent_wormholes > 0 and (new_row, new_col) not in wormholes:
                probability_wormhole = n_adjacent_wormholes * (proba_remaining / number_of_wormholes)
                for wormholes_to_update in wormholes:
                    transitions_probability = [
                        (
                            current_next_state,
                            probability_wormhole if current_next_state == wormholes_to_update else original_prob,
                            None,
                        )
                        for current_next_state, original_prob, _ in transitions_probability
                    ]

            # Lastly, the case where the second option is not a Cliff, edge, start or wormhole :
            transitions_probability = [
                (
                    (next_state, proba_remaining, None)
                    if grid[next_state[0]][next_state[1]] != Cell.START
                    and grid[next_state[0]][next_state[1]] != Cell.WORMHOLE
                    and (next_state[0], next_state[1]) != (new_row, new_col)
                    else (next_state, original_prob, None)
                )
                for next_state, original_prob, _ in transitions_probability
            ]

        return transitions_probability

    def motion_for_swamp(self, state: State, action: Action, next_state: State):
        grid = np.array(self.grid)
        all_possible_outcomes = self.get_possible_moves_even_non_admissible((state[0], state[1]))
        transitions_probability = [(next_state, None, None) for next_state in all_possible_outcomes]

        row = state[0]
        col = state[1]

        # Initialize new_row and new_col
        new_row, new_col = row, col

        if action == Action.NORTH:
            new_row = row - 1
        elif action == Action.SOUTH:
            new_row = row + 1
        elif action == Action.WEST:
            new_col = col - 1
        elif action == Action.EAST:
            new_col = col + 1

        if action == Action.ABANDON:
            return (self.get_start(), 1, None)

        elif action != Action.ABANDON:

            ## First, do the case of an adjacant wormhole
            proba = 0.5
            proba_remaining = (1 - 0.75) / 3
            wormholes, number_of_wormholes = self.get_wormholes()
            n_adjacent_wormholes = self.find_number_of_adjacant_wormholes(state)

            if (new_row, new_col) in wormholes:
                probability_wormhole = (1 / number_of_wormholes) * (
                    proba + (n_adjacent_wormholes - 1) * proba_remaining
                )

                for wormholes_to_update in wormholes:
                    transitions_probability = [
                        (
                            current_next_state,
                            probability_wormhole if current_next_state == wormholes_to_update else original_prob,
                            None,
                        )
                        for current_next_state, original_prob, _ in transitions_probability
                    ]

            # If the first movement is on the start case :

            elif grid[new_row][new_col] == Cell.START:
                transitions_probability = [
                    (next_state, proba, None) if next_state == self.get_start() else (next_state, original_prob, None)
                    for next_state, original_prob, _ in transitions_probability
                ]

            else:
                ## If the next move is not a wormhole
                transitions_probability = [
                    (next_state, proba, None) if next_state == (new_row, new_col) else (next_state, original_prob, None)
                    for next_state, original_prob, _ in transitions_probability
                ]

            ## probability to reach the start again which is equal to number of cliffs or edges
            ## Added for the swamp :
            if grid[new_row][new_col] != Cell.START:
                proba_brake_need_go_to_start = 0.05
                number_outside_cases_or_cliff_ = self.number_outside_cases_or_cliff(state)
                number_access_to_start_ = self.number_access_to_start(state)
                proba_start = (
                    number_outside_cases_or_cliff_ + number_access_to_start_
                ) * proba_remaining + proba_brake_need_go_to_start
                # Dont forget to add the case where it when directly to the start (0.75 chance to go start)
                transitions_probability = [
                    (
                        (next_state, (proba_start + original_prob if original_prob is not None else proba_start), None)
                        if next_state == self.get_start()
                        else (next_state, original_prob if original_prob is not None else 0.0, None)
                    )
                    for next_state, original_prob, _ in transitions_probability
                ]

            ## Update if no wormhole in the next move but worlhole in the neighborhood
            if n_adjacent_wormholes > 0 and (new_row, new_col) not in wormholes:
                probability_wormhole = n_adjacent_wormholes * (proba_remaining / number_of_wormholes)
                for wormholes_to_update in wormholes:
                    transitions_probability = [
                        (
                            current_next_state,
                            probability_wormhole if current_next_state == wormholes_to_update else original_prob,
                            None,
                        )
                        for current_next_state, original_prob, _ in transitions_probability
                    ]

            # Lastly, the case where the second option is not a Cliff, edge, start or wormhole :
            transitions_probability = [
                (
                    (next_state, proba_remaining, None)
                    if grid[next_state[0]][next_state[1]] != Cell.START
                    and grid[next_state[0]][next_state[1]] != Cell.WORMHOLE
                    and (next_state[0], next_state[1]) != (new_row, new_col)
                    else (next_state, original_prob, None)
                )
                for next_state, original_prob, _ in transitions_probability
            ]

            ## Case added in case the robot stays at the same place :
            transitions_probability.append(((state[0], state[1]), 0.2, None))

        return transitions_probability

    def get_transition_prob(self, state: State, action: Action, next_state: State) -> float:
        """Returns P(next_state | state, action)"""
        grid = np.array(self.grid)
        if grid[state[0]][state[1]] == Cell.SWAMP:
            transitions_probability = self.motion_for_swamp(state, action, next_state)
            return transitions_probability

        elif (
            grid[state[0]][state[1]] == Cell.GRASS
            or grid[state[0]][state[1]] == Cell.WORMHOLE
            or grid[state[0]][state[1]] == Cell.START
        ):
            transitions_probability = self.motion_for_Grass_and_wormholes(state, action, next_state)
            return transitions_probability
        elif grid[state[0]][state[1]] == Cell.GOAL:
            transitions_probability = ((state), 1, None)
            return transitions_probability
        elif action == Action.ABANDON:
            return (self.get_start(), 1, None)

    def stage_reward(self, state: State, action: Action, next_state: State) -> float:
        grid = np.array(self.grid)

        if action == Action.ABANDON:
            new_reward = -10
            return new_reward

        if self.number_outside_cases_or_cliff(state) > 0 and grid[next_state[0]][next_state[1]] == Cell.START:
            new_reward = -10
            if grid[state[0], state[1]] == Cell.GRASS:
                new_reward = new_reward - 1
                return new_reward
            elif grid[state[0], state[1]] == Cell.SWAMP:
                new_reward = new_reward - 2
                return new_reward
            elif grid[state[0], state[1]] == Cell.WORMHOLE:
                new_reward = new_reward - 1
                return new_reward

        if grid[state[0], state[1]] == Cell.GRASS:
            new_reward = -1
            return new_reward

        elif grid[state[0], state[1]] == Cell.SWAMP:
            new_reward = -2
            return new_reward

        elif grid[next_state[0]][next_state[1]] == Cell.WORMHOLE and grid[state[0], state[1]] == Cell.GRASS:
            new_reward = -1
            return new_reward

        elif grid[next_state[0]][next_state[1]] == Cell.WORMHOLE and grid[state[0], state[1]] == Cell.SWAMP:
            new_reward = -2
            return new_reward

        elif grid[next_state[0]][next_state[1]] == Cell.WORMHOLE and grid[state[0], state[1]] == Cell.START:
            new_reward = -2
            return new_reward

        elif grid[state[0], state[1]] == Cell.WORMHOLE:
            new_reward = -1
            return new_reward

        elif grid[state[0], state[1]] == Cell.START:
            new_reward = -1
            return new_reward

        elif grid[next_state[0]][next_state[1]] == Cell.GOAL:
            new_reward = 50
            return new_reward


class GridMdpSolver(ABC):
    @staticmethod
    @abstractmethod
    def solve(grid_mdp: GridMdp) -> tuple[ValueFunc, Policy]:

        pass
