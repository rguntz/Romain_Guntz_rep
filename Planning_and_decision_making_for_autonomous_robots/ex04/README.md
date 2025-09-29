# Grid MDP Solver

This project implements **Markov Decision Process (MDP)** solvers for grid-based environments. It provides classes and algorithms to model a robot navigating a grid with various types of cells and rewards.

## Key Components

- **GridMdp**: 
  - Represents the grid environment.
  - Handles movement rules, valid actions, and stochastic transitions for different cell types (Grass, Swamp, Wormhole, Cliff, Start, Goal).
  - Computes stage rewards and transition probabilities.

- **GridMdpSolver**: Abstract base class for MDP solvers.

- **ValueIteration**:
  - Implements the value iteration algorithm to compute the optimal policy and value function.
  - Iteratively updates state values until convergence using the Bellman equation.

- **PolicyIteration**:
  - Implements the policy iteration algorithm to compute the optimal policy and value function.
  - Alternates between **policy evaluation** (computing state values for a fixed policy) and **policy improvement** (updating the policy based on state values) until convergence.

## Features

- Supports stochastic movements and complex grid interactions (wormholes, swamps, cliffs).
- Computes both **optimal value functions** and **optimal policies**.
- Includes utilities for measuring performance of the solvers.

## Usage

1. Define a grid using the  class.
2. Choose a solver ( or ).
3. Call  to get the optimal value function and policy.

