# PDM4AR: Spaceship Agent and Planner

This project implements a spaceship agent and planner for the PDM4AR simulation environment. It provides a framework for trajectory planning, discretization of dynamics, and control for a spaceship navigating among planets and satellites.

## Overview of Files

- **agent.py**  
  Defines the  class that interacts with the simulator.  
  - Initializes the agent with environment information (planets, satellites, initial state).  
  - Computes an initial trajectory using the .  
  - Returns commands at each simulation step, with potential for replanning.

- **discretization.py**  
  Implements methods for discretizing the continuous-time spaceship dynamics.  
  -  and  classes approximate nonlinear dynamics over time.  
  - Provides integration methods for piecewise or full trajectory simulation.

- **planner.py**  
  Implements the  class responsible for trajectory planning.  
  - Uses SCvx (Successive Convexification) or similar convex optimization methods.  
  - Defines optimization variables, constraints, and objectives.  
  - Slack variables are introduced to relax hard constraints, such as collisions with walls, ensuring feasible solutions.  
  - Computes an initial guess, performs discretization, and solves for a trajectory.

- **spaceship.py**  
  Defines the  class representing the spaceship dynamics.  
  - Uses symbolic variables for states, inputs, and parameters.  
  - Provides functions to compute dynamics and their Jacobians for optimization.

## High-Level Workflow

1. The agent () is initialized with environmental parameters.  
2. On episode start, it uses the planner () to compute a trajectory from the initial state to a goal state.  
3. The discretization classes ( / ) approximate the continuous dynamics for optimization.  
4. Slack variables are used to avoid infeasible solutions when constraints like walls or obstacles are present.  
5. At each simulation step, the agent provides commands to track the trajectory, with the option to replan if deviations occur.  
6. The dynamics () provide the necessary system equations and derivatives for planning and optimization.

This setup enables realistic trajectory planning and control of a spaceship in dynamic environments with moving obstacles.


https://github.com/user-attachments/assets/81df91b4-d2df-4d04-9282-375b162a4b97

https://github.com/user-attachments/assets/ea813dee-f071-4939-88bc-1d905e2d99d9


