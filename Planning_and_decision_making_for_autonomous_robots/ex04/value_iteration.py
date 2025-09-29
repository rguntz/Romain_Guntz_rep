import numpy as np
from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import Policy, ValueFunc, Cell
from pdm4ar.exercises_def.ex04.utils import time_function


import numpy as np
import ast


class ValueIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> tuple[ValueFunc, Policy]:
        value_func = np.zeros_like(grid_mdp.grid).astype(float)  # Initialization of the zero matrices
        policy = np.zeros_like(grid_mdp.grid).astype(int)  # Initialization of the zero matrices

        gamma = grid_mdp.gamma  # Discount factor
        theta = 10e-2  # Convergence threshold

        GridMdp_object = grid_mdp

        V = np.zeros_like(GridMdp_object.grid).astype(float)
        policy = np.zeros_like(GridMdp_object.grid).astype(object)  # Policy stores the best action for each state

        grid_test = GridMdp_object.grid

        data = []
        data_matrix = np.zeros_like(GridMdp_object.grid, dtype=object)

        for j in range(0, 1):  # Infinite loop, terminates when convergence is reached
            delta = 0
            new_V = np.copy(V)
            for x in range(V.shape[0]):
                for y in range(V.shape[1]):
                    actions_list = []
                    if grid_test[x][y] != Cell.CLIFF:
                        v = V[x, y]
                        # Calculate the expected utility for each action
                        action_values = []
                        action_name_list = []
                        for action in GridMdp_object.get_actions_to_do((x, y)):
                            get_transition_prob_object = GridMdp_object.get_transition_prob((x, y), action, (0, 0))

                            if not isinstance(get_transition_prob_object, list):
                                get_transition_prob_object = [get_transition_prob_object]

                            transition_list = list(get_transition_prob_object)
                            transition_list.append(action)
                            transition_list.append((x, y))
                            data.append(transition_list)
                            actions_list.append(transition_list)

                        data_matrix[x][y] = actions_list

        gamma = 0.9  # Discount factor
        theta = 10e-2  # Convergence threshold

        GridMdp_object = grid_mdp

        V = np.zeros_like(GridMdp_object.grid).astype(float)
        policy = np.zeros_like(GridMdp_object.grid).astype(object)  # Policy stores the best action for each state

        grid_test = GridMdp_object.grid

        while True:  # Infinite loop, terminates when convergence is reached
            delta = 0
            new_V = np.copy(V)
            for x in range(V.shape[0]):
                for y in range(V.shape[1]):
                    if grid_test[x][y] != Cell.CLIFF:
                        v = V[x, y]
                        # Calculate the expected utility for each action
                        action_values = []
                        action_name_list = []

                        for action in GridMdp_object.get_actions_to_do((x, y)):
                            proba_tot = 0

                            transition_pro_for_x_y = data_matrix[x][y]

                            for item in transition_pro_for_x_y:
                                item_to_work = item.copy()
                                item_to_work.pop()
                                if item_to_work[-1] == action:
                                    item_to_work.pop()
                                    get_transition_prob_object = item_to_work.copy()
                                    break

                            for transition in get_transition_prob_object:

                                probability = transition[1]  # Get the probability of the new position
                                coordinates = transition[0]  # Get the coordinates of the new position
                                new_x = coordinates[0]
                                new_y = coordinates[1]
                                R = GridMdp_object.stage_reward((x, y), action, (new_x, new_y))
                                proba_tot = proba_tot + probability * (R + gamma * V[new_x, new_y])

                            action_values.append(proba_tot)
                            action_name_list.append(action)

                        # Update the value of state (x, y)
                        new_V[x, y] = max(action_values)
                        policy[x, y] = action_name_list[np.argmax(action_values)]
                        delta = max(delta, abs(v - new_V[x, y]))

            V = new_V
            # Convergence condition
            if delta < theta:
                break

        # todo implement here

        V = np.array(V, dtype=np.float64)
        # Ensure `policy` is a numpy array of integers
        policy = np.array(policy, dtype=np.int64)

        # Print for debugging if necessary

        return V, policy
