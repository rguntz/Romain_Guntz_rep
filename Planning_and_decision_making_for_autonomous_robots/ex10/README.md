# Frenet-Frame Trajectory Planner for Autonomous Driving

This project implements a **real-time optimal trajectory planner** for autonomous vehicles in the **CommonRoad/DgCommons simulation stack**, using **Frenet-frame path planning** with **quintic and quartic polynomial trajectories**. The agent avoids both static road boundaries and dynamic obstacles (other vehicles) while tracking a reference lane toward a goal.

## 🚗 Core Features

- **Frenet Optimal Trajectory Planning**: Generates candidate trajectories in the Frenet frame (longitudinal `s` and lateral `d` coordinates) using:
  - **Quintic polynomials** for lateral motion (smooth lane changes or centering).
  - **Quartic polynomials** for longitudinal motion (speed profile optimization).
- **Collision-Aware Planning**:
  - Predicts future positions of dynamic obstacles based on their current speed and heading.
  - Uses **Shapely geometries** to perform precise **polygon-based collision checks** between the ego vehicle and obstacles over the planning horizon.
  - Considers **static road boundaries** (e.g., lane markings, barriers) as line obstacles.
- **Adaptive Behavior**:
  - Adjusts planning parameters (e.g., prediction horizon, target speed) based on traffic density.
  - Slows down when vehicles ahead are moving slowly ("crowded" mode).
- **Reference Path Tracking**:
  - Builds a **2D cubic spline** from the goal lane’s control points to serve as the reference path.
  - Converts vehicle pose to Frenet coordinates using closest-point projection.
- **Low-Level Control**:
  - Uses a **PID controller** to track the planned trajectory’s heading and speed.

## 📂 Project Structure

- `agent.py`: Main agent class (`Pdm4arAgent`) integrating planning, collision checking, and control.
- Trajectory generation logic:
  - `quintic_polynomials_planner.py`: Quintic polynomial trajectory generation (lateral).
  - `frenet_optimal_trajectory.py`: Reference implementation (from Atsushi Sakai) adapted for this project.
  - `cubic_spline_planner.py`: 1D/2D cubic spline utilities for reference path smoothing.
- Key internal components:
  - `CubicSpline2D`: Converts waypoints into a smooth reference curve with curvature/yaw.
  - `Frenet_path`: Data structure for candidate trajectories.
  - `check_paths_computation`: Performs dynamic and static collision checks using Shapely polygons.
  - `frenet_optimal_planning_computation`: Orchestrates sampling, cost evaluation, and selection of the best safe trajectory.

## ⚙️ Planning Parameters (Tunable)

| Parameter             | Description                          |
|----------------------|--------------------------------------|
| `MAX_SPEED`          | Maximum allowed vehicle speed        |
| `MAX_ACCEL`          | Maximum longitudinal acceleration    |
| `MAX_CURVATURE`      | Maximum turning curvature            |
| `MAX_ROAD_WIDTH`     | Lateral search range around centerline |
| `MINT` / `MAXT`      | Min/max prediction horizon (seconds) |
| `TARGET_SPEED`       | Desired cruising speed               |
| Cost weights (`KJ`, `KT`, `KLAT`, `KLON`) | Balance smoothness, time, lateral deviation, and speed tracking |

## 🧠 How It Works (High-Level Flow)

1. **Initialization**: Extract goal lane, extend it, and build a cubic spline reference path.
2. **At each simulation step**:
   - Convert ego pose to Frenet coordinates (`s`, `d`).
   - Sample lateral goals (`d`) and time horizons (`T`).
   - For each combination, generate lateral (quintic) and longitudinal (quartic) trajectories.
   - Convert Frenet paths to global (x, y) coordinates.
   - Predict future positions of dynamic obstacles.
   - Perform **polygon-based collision checks** with both static and dynamic obstacles.
   - Evaluate cost (smoothness + deviation + speed error) and select the **lowest-cost safe path**.
   - If no safe path is found, recursively relax constraints (e.g., shorten horizon).
3. **Control**: Use PID to track the selected trajectory’s heading and speed.

## 📌 Notes

- Designed for **DgCommons / CommonRoad** environments.
- Relies on **Shapely** for geometric operations and **NumPy** for numerical computation.
- Includes optional live plotting for debugging (disabled by default in production).

## 📚 References

- Werling, M., Ziegler, J., Kammel, S., & Thrun, S. (2010). *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame*.
- Atsushi Sakai’s [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (base implementation adapted).

---
*Developed for autonomous driving research and simulation within the PDM4AR framework.*



https://github.com/user-attachments/assets/ece26419-cc52-4f2b-be4f-577824dce8cf



https://github.com/user-attachments/assets/c8e90c83-8a0d-420e-88b0-96e5a4db77e2





