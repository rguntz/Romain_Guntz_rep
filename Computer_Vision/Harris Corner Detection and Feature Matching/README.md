# Harris Corner Detection and Feature Matching

This project implements **Harris corner detection** and **feature matching** between image pairs using hand-crafted descriptors (image patches). It demonstrates fundamental computer vision techniques for keypoint detection, descriptor extraction, and matching strategies.

## Features

- Harris corner detection with configurable parameters
- Keypoint filtering to ensure valid patch extraction
- Patch-based descriptor extraction (9×9 pixel neighborhoods)
- Three matching strategies:
  - One-way nearest neighbor
  - Mutual nearest neighbor
  - Ratio test (Lowe's method)
- Visualization of detected corners and matches
