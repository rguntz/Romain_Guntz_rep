# Master's and Personal Projects

This repository contains a collection of my Master's and personal projects, showcasing work in algorithms, robotics, optimization, and autonomous systems.

## Backgammon (Tavli) Algorithm

Implementation of a backgammon game algorithm coded in C.

<div align="center">
  <img width="188" height="256" alt="Backgammon algorithm screenshot" src="https://github.com/user-attachments/assets/8151aeea-9dd9-4f75-8264-f94a8f6de996" />
</div>

## Planning and Decision Making for Autonomous Robots

Coursework projects focusing on motion planning and control for autonomous robots. Includes:
- **Searching Algorithms**: Techniques for pathfinding and navigation.
- **Dubins Car Model**: Planning paths for vehicles with constrained turning radius.
- **Markov Decision Processes (MDP)**: Modeling robot navigation as a sequential decision-making problem. The robot chooses actions to maximize long-term rewards while accounting for uncertainties in the environment.
- **Rocket Landing Optimization**: Successive Convexification (SCvx) approach for rocket landing, including slack variables to avoid collisions with obstacles like walls.
- **Autonomous Vehicle Trajectory Planning**: Frenet-frame optimal trajectory generation using quintic lateral and quartic longitudinal polynomials, with dynamic obstacle avoidance via polygon-based collision checking and adaptive speed control in crowded traffic.

**Rocket Landing Optimization**

<div align="center">

https://github.com/user-attachments/assets/2174a67b-29c1-4a6a-ac9c-27d9a303f667

</div>

**Autonomous Vehicle Trajectory Planning**

<div align="center">

https://github.com/user-attachments/assets/c44900ee-f02f-4179-bd1c-214c3f7087ad

</div>

## Computer Vision

Project from the Computer Vision class (Prof. Dr. Marc Pollefeys).
- **Harris Corner Detection and Feature Matching**: Implements Harris corner detection and feature matching between image pairs using hand-crafted descriptors (image patches).
- **SegNet Light**: A compact implementation of the SegNet architecture for semantic segmentation, designed for digit/background segmentation tasks.
- **Structure-from-Motion (SfM) Pipeline**: An incremental Structure-from-Motion pipeline that reconstructs a 3D scene from unordered 2D images using only feature matches and camera intrinsics.

**Harris Detector**

<div align="center">
  <img width="256" height="256" alt="Harris corner detection on a block scene" src="https://github.com/user-attachments/assets/0c6f2d40-5c37-46be-ac90-1c17b613909f" />
</div>

**Structure-from-Motion**

<div align="center">
  <img width="636" height="206" alt="Structure-from-Motion reconstruction" src="https://github.com/user-attachments/assets/7baba738-b6d0-4901-8774-888c2a57f18f" />
</div>

## German Sentence Generator with Speech Synthesis

A web application that generates natural German sentences from a set of words, provides English translations, and synthesizes German speech. Features adaptive learning: words the user doesn't know are prioritized in future rounds for personalized vocabulary practice.

<div align="center">
  <img width="755" height="785" alt="German Sentence Generator app screenshot" src="https://github.com/user-attachments/assets/2f43551e-2947-40d6-8102-573c54b5360c" />
</div>

## Real-World Sample-Efficient Reinforcement Learning

My current project focuses on dexterous manipulation for assembly tasks. The approach builds on ideas introduced in the paper "Imitation Bootstrapped Reinforcement Learning" by Hengyuan Hu et al. It begins by learning a policy from expert demonstrations, which is then reused as a guiding prior to improve exploration efficiency and learning stability during reinforcement learning, in parallel with directly incorporating demonstrations into the RL pipeline. Using this approach, the policy learned the assembly task in about one hour of real-world practice — with real-world rollouts and improvement, and without relying on simulation.

<div align="center">

https://github.com/user-attachments/assets/6d233441-b757-4c48-aed9-c1964219154b

</div>

*Note: this is the first time I am running the RL policy — its motion is going to become smoother!*

---

Feel free to explore the folders to see the implementations and examples of usage.
