# Technical Case Study: Asset Visual Coverage Analysis

## Objective 

This calculates the visual coverage of an asset. Specifically, it determines the percentage of the asset, represented as a point cloud, that is covered by images based on camera trajectory and intrinsic parameters.

## Methodology 

### Camera Coordinate Frame

Upon receiving input data, the camera coordinate frame is set as shown below:
![alt text](https://facebookresearch.github.io/projectaria_tools/assets/images/camera3d-coordinate-frame-8e7eb3a8462f8402724205da4332725a.png)

Given roll, pitch, and yaw values are assumed to be provided in radians.

### Point Cloud Projection 

The program projects each point in the point cloud onto a 2D plane using camera poses and intrinsic parameters. Instead of using the field of view (FOV) to determine point validity, the image width and height are used as criteria for valid point projections.

## Code Structure Overview

* ```load_camera_intrinsics(json_path)```: Loads camera intrinsics, including focal lengths (fx, fy), principal points (cx, cy), and field of view.

* ```load_point_cloud(pcd_path)```: Reads a point cloud from a PCD file into a NumPy array.

* ```load_trajectory(csv_path)```: Loads camera trajectory data from a CSV file, including timestamp, position, and orientation (roll, pitch, yaw). The file ```trajectory_s.csv``` includes full camera poses, while ```trajectory_pose1.csv``` contains only one pose, and ```trajectory_pose2.csv``` includes two poses.

* ```calculate_rotation_matrix(roll, pitch, yaw)```: Computes a rotation matrix for transforming points to the camera coordinate frame.

* ```compute_coverage(point_cloud, trajectory, camera_intrinsics, image_path)```: Projects points to determine coverage and computes the percentage of points visible from the images.

* ```visualization(covered_points, uncovered_points, cameras)```: Displays covered and uncovered points with camera coordinate frames.

## Prerequisites

Ensure the following Python packages are installed before running the code:
```
pip install numpy opencv-python open3d pandas
```

## Running Asset Visual Coverage 

Run the script as follows:
```
python3 asset_coverage.py
```
Modify the paths to match your local files:

* ```asset.pcd``` (point cloud)

* ```trajectory.csv``` (camera trajectory)

* ```camera_intrinsic.json``` (camera intrinsic parameters)

## Result 

A visualization function is provided to display the results. The output highlights the coverage of points in the point cloud:

  * **Red**: Covered point
  * **Black**: Uncovered point 
 ![Screenshot from 2025-01-21 12-42-06](https://github.com/user-attachments/assets/2bc5efa2-cb58-4eb2-ba28-9a8632390aa8)

## Future work

Currently, the implementation assumes that the camera can observe all features in front of it, without accounting for occlusions or obstructions. Future improvements should incorporate occlusion handling for more accurate coverage analysis.

