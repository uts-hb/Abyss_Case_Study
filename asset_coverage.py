import numpy as np
import cv2
import json
import open3d as o3d
import pandas as pd
import math

# Load camera intrinsics from JSON file
def load_camera_intrinsics(json_path):
    with open(json_path, 'r') as f:
        data = json.load(f)
    fx, fy, cx, cy = data['fx'], data['fy'], data['cx'], data['cy']
    hor_fov, ver_fov = data['hor_fov'], data['ver_fov']
    return fx, fy, cx, cy, hor_fov, ver_fov

# Load point cloud from PCD file
def load_point_cloud(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    return np.asarray(pcd.points)

# Load trajectory data from CSV file
def load_trajectory(csv_path):
    return pd.read_csv(csv_path, names=['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'], skiprows=1)

# Calculate rotation matrix from roll, pitch, and yaw angles
def calculate_rotation_matrix(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    R = R_x @ R_y @ R_z
    return R_z@R_y@R_x


# Visualization of seen and unseen points with camera coordinate frames
def visualization(covered_points, uncovered_points, cameras):
    covered_pc = o3d.geometry.PointCloud()
    uncovered_pc = o3d.geometry.PointCloud()
    
    if len(covered_points) > 0:
        covered_pc.points = o3d.utility.Vector3dVector(covered_points)
        covered_pc.paint_uniform_color([1, 0, 0])  # Red for covered points

    if len(uncovered_points) > 0:
        uncovered_pc.points = o3d.utility.Vector3dVector(uncovered_points)
        uncovered_pc.paint_uniform_color([0, 0, 0])  # Black for uncovered points

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(covered_pc)
    vis.add_geometry(uncovered_pc)
    for camera in cameras:
        vis.add_geometry(camera)
    vis.run()
    vis.destroy_window()


# Compute visual-coverage of the asset
def compute_coverage(point_cloud, trajectory, camera_intrinsics, image_path):
    fx, fy, cx, cy, hor_fov, ver_fov = camera_intrinsics
    # half_hor_fov_rad = np.radians(hor_fov / 2)
    # half_ver_fov_rad = np.radians(ver_fov / 2)

    # Image size
    image_shape = cv2.imread(image_path).shape
    image_width, image_height = image_shape[1], image_shape[0]
    
    covered_points = set()

    cameras = []
    for _, row in trajectory.iterrows():
        x, y, z, roll, pitch, yaw = row[1:]
        rotation_matrix = calculate_rotation_matrix(roll, pitch, yaw)
        translation_vector = np.array([x, y, z])
        camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = translation_vector
        cameras.append(camera.transform(transform))

        for point in point_cloud:
            
            # Transform point to camera coordinate system
            camera_point = np.dot(rotation_matrix.T, point - translation_vector)
            
            # Check if the point is in front of the camera
            if camera_point[2] <= 0:
                continue
            
            # Project the point to the image plane
            u = fx * camera_point[0] / camera_point[2] + cx
            v = fy * camera_point[1] / camera_point[2] + cy

            # Check if the projected point is inside the image
            if 0 <= u < image_width and 0 <= v < image_height:
                covered_points.add(tuple(point))

    # calculate coverage percentage
    covered_points_array = np.array(list(covered_points))
    uncovered_points_array = np.array([p for p in point_cloud if tuple(p) not in covered_points])

    len_covered_points = len(covered_points)
    len_total_points = len(point_cloud)
    coverage_percentage = (len_covered_points / len_total_points) * 100
    print(f"Coverage: {coverage_percentage:.2f}%")

    # Visualization
    visualization(covered_points_array, uncovered_points_array, cameras)
    

if __name__ == "__main__":
    
    # Change the paths to the actual paths of the files
    point_cloud_path = '/home/sewer/Abyss_new/mock_data/asset.pcd'
    trajectory_path = '/home/sewer/Abyss_new/mock_data/trajectory_2.csv'
    intrinsics_path = '/home/sewer/Abyss_new/mock_data/camera_intrinsics.json'
    image_path = '/home/sewer/Abyss_new/mock_data/images/image_0000.jpg'
    
    point_cloud_array = load_point_cloud(point_cloud_path)
    trajectory_data = load_trajectory(trajectory_path)
    camera_intrinsics = load_camera_intrinsics(intrinsics_path)

    compute_coverage(point_cloud_array, trajectory_data, camera_intrinsics, image_path)   
    