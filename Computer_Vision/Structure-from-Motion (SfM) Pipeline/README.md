Structure-from-Motion (SfM) Pipeline

This project implements an incremental Structure-from-Motion pipeline that reconstructs a 3D scene from unordered 2D images using only feature matches and camera intrinsics.

Files:
- corrs.py: Manages 2D–3D correspondences.
- geometry.py: Essential matrix estimation, pose decomposition, triangulation, and PnP.
- line_fitting.py: RANSAC line fitting demo (educational).
- sfm.py: Main SfM loop—initializes from an image pair and incrementally registers new views.
- impl/: Utility functions (DLT, I/O, visualization, etc.).

Requirements: Python 3.7+, NumPy, Matplotlib.

To run:
1. Place images (0000.png–0009.png), matches, and K.txt in ../data/
2. Run: python sfm.py

The pipeline:
- Estimates relative pose from an initial image pair
- Triangulates 3D points
- Registers new images via 2D–3D correspondences
- Triangulates additional points and updates reconstruction
- Visualizes 3D points and camera poses

Designed for educational purposes—illustrates core multi-view geometry concepts (epipolar geometry, DLT, RANSAC, incremental SfM) without external dependencies. Not optimized for large-scale use.


<img width="640" height="210" alt="Capture d’écran 2025-09-30 à 21 47 10" src="https://github.com/user-attachments/assets/013908e9-439a-497d-aca5-987c474a0a15" />


