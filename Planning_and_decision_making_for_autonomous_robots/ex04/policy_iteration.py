import numpy as np

from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import ValueFunc, Policy, Cell
from pdm4ar.exercises_def.ex04.utils import time_function


class PolicyIteration(GridMdpSolver):
    @staticmethod
    @time_function
    def solve(grid_mdp: GridMdp) -> tuple[ValueFunc, Policy]:
        gamma = grid_mdp.gamma  # Discount factor
        theta = 10e-2  # Convergence threshold

        GridMdp_object = grid_mdp
        grid_test = GridMdp_object.grid

        # Initialization
        V = np.zeros_like(GridMdp_object.grid).astype(float)
        policy = np.zeros_like(GridMdp_object.grid).astype(object)  # Initial policy

        # Policy evaluation step
        def policy_evaluation(policy, V):
            while True:
                delta = 0
                new_V = np.copy(V)
                for x in range(V.shape[0]):
                    for y in range(V.shape[1]):
                        if grid_test[x][y] != Cell.CLIFF:
                            action = policy[x, y]
                            action_value = 0

                            get_transition_prob_object = GridMdp_object.get_transition_prob((x, y), action, (0, 0))

                            if not isinstance(get_transition_prob_object, list):
                                get_transition_prob_object = [get_transition_prob_object]

                            for transition in get_transition_prob_object:
                                probability = transition[1]
                                new_x, new_y = transition[0]
                                R = GridMdp_object.stage_reward((x, y), action, (new_x, new_y))
                                action_value += probability * (R + gamma * V[new_x, new_y])

                            new_V[x, y] = action_value
                            delta = max(delta, abs(V[x, y] - new_V[x, y]))

                V = new_V
                if delta < theta:
                    break
            return V

        # Policy improvement step
        def policy_improvement(V, data):
            new_policy = np.zeros_like(policy)
            for x in range(V.shape[0]):
                for y in range(V.shape[1]):
                    if grid_test[x][y] != Cell.CLIFF:
                        action_values = []
                        action_name_list = []
                        for action in GridMdp_object.get_actions_to_do((x, y)):

                            transition_pro_for_x_y = data_matrix[x][y]

                            for item in transition_pro_for_x_y:
                                item_to_work = item.copy()
                                item_to_work.pop()
                                if item_to_work[-1] == action:
                                    item_to_work.pop()
                                    get_transition_prob_object = item_to_work.copy()
                                    break

                            action_value = 0
                            for transition in get_transition_prob_object:
                                probability = transition[1]
                                new_x, new_y = transition[0]
                                R = GridMdp_object.stage_reward((x, y), action, (new_x, new_y))
                                action_value += probability * (R + gamma * V[new_x, new_y])

                            action_values.append(action_value)
                            action_name_list.append(action)

                        new_policy[x, y] = np.int64(action_name_list[np.argmax(action_values)])

            return new_policy

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

        # Main loop for Policy Iteration
        while True:
            # Policy evaluation
            V = policy_evaluation(policy, V)

            # Policy improvement
            new_policy = policy_improvement(V, data)

            # Check for convergence (if policy does not change)
            if np.array_equal(new_policy, policy):
                break

            policy = new_policy

        # Convert V and policy to the required data types
        V = np.array(V, dtype=np.float64)
        policy = np.array(policy, dtype=np.int64)

        # Print for debugging if necessary
        print(V)
        print(policy)

        return V, policy
