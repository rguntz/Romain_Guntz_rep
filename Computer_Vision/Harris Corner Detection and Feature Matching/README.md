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


<img width="1306" height="490" alt="match_ratio" src="https://github.com/user-attachments/assets/efb69035-0caf-47ce-8eef-3649c592c8bd" />

<img width="256" height="256" alt="blocks_harris" src="https://github.com/user-attachments/assets/5192607d-2e4a-4147-9f6f-ddb893812c5b" />
