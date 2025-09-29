# Path Planning for Non-Holonomic Vehicles

This project provides algorithms for planning feasible paths for car-like robots in 2D space, considering motion constraints such as a minimum turning radius and non-holonomic behavior.

It implements two main path planners:

- Dubins Paths: Forward-only paths combining circular arcs and straight segments to connect two poses optimally.
- Reeds-Shepp Paths: Extensions of Dubins paths that allow both forward and backward motion, enabling more flexible trajectories in constrained environments.

The system handles complex path geometries, including sequences of curves and tangent segments, to compute smooth and realistic trajectories for autonomous navigation.
