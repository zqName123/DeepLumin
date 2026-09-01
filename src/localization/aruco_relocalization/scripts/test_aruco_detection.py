#!/usr/bin/env python3
"""
ArUco 检测和位姿解算测试脚本
测试数据：/home/hhy/2004ros/data/4_video_office_large
"""

import cv2
import numpy as np
import glob
import os
import yaml
from pathlib import Path

def load_camera_params(yaml_path):
    """从 camera.yml 加载相机参数"""
    with open(yaml_path, 'r') as f:
        content = f.read()

    # 解析 YAML（处理 OpenCV 特殊格式）
    # camera_matrix
    import re
    K_match = re.search(r'camera_matrix:.*?data:\s*\[([\d.e+\-\s,]+)\]', content, re.DOTALL)
    if K_match:
        K_data = [float(x.strip()) for x in K_match.group(1).split(',')]
        camera_matrix = np.array(K_data).reshape(3, 3)
    else:
        raise ValueError("Cannot parse camera_matrix")

    # distortion_coefficients
    D_match = re.search(r'distortion_coefficients:.*?data:\s*\[([\d.e+\-\s,]+)\]', content, re.DOTALL)
    if D_match:
        D_data = [float(x.strip()) for x in D_match.group(1).split(',')]
        dist_coeffs = np.array(D_data)
    else:
        raise ValueError("Cannot parse distortion_coefficients")

    return camera_matrix, dist_coeffs

def test_aruco_detection(image_dir, camera_yaml, marker_size=0.123):
    """测试 ArUco 检测和位姿解算"""

    # 加载相机参数
    print(f"Loading camera parameters from: {camera_yaml}")
    camera_matrix, dist_coeffs = load_camera_params(camera_yaml)
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs}")

    # 获取所有图像
    image_files = sorted(glob.glob(os.path.join(image_dir, "image-*.jpg")))
    print(f"\nFound {len(image_files)} images")

    if not image_files:
        print("No images found!")
        return

    # 创建 ArUco 检测器
    # 使用 ARUCO 原始字典
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # 统计信息
    total_markers = 0
    detected_images = 0
    marker_ids_seen = set()
    all_detections = []

    print("\n" + "="*80)
    print("Starting ArUco detection test...")
    print("="*80)

    for idx, image_file in enumerate(image_files):
        # 读取图像
        image = cv2.imread(image_file)
        if image is None:
            print(f"Failed to read: {image_file}")
            continue

        # 检测 marker
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

        if ids is None or len(ids) == 0:
            continue

        detected_images += 1
        total_markers += len(ids)

        # 估计位姿
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, dist_coeffs
        )

        # 记录检测结果
        frame_name = os.path.basename(image_file)
        for i, marker_id in enumerate(ids.flatten()):
            marker_ids_seen.add(marker_id)

            # 计算重投影误差
            obj_points = np.array([
                [-marker_size/2, -marker_size/2, 0],
                [ marker_size/2, -marker_size/2, 0],
                [ marker_size/2,  marker_size/2, 0],
                [-marker_size/2,  marker_size/2, 0]
            ])

            projected, _ = cv2.projectPoints(obj_points, rvecs[i], tvecs[i],
                                            camera_matrix, dist_coeffs)
            projected = projected.reshape(-1, 2)

            # 计算角点误差
            corner_errors = []
            for j in range(4):
                dx = projected[j][0] - corners[i][0][j][0]
                dy = projected[j][1] - corners[i][0][j][1]
                corner_errors.append(np.sqrt(dx*dx + dy*dy))
            mean_error = np.mean(corner_errors)

            # 计算距离
            distance = np.linalg.norm(tvecs[i])

            detection_info = {
                'frame': frame_name,
                'marker_id': int(marker_id),
                'tvec': tvecs[i].tolist(),
                'rvec': rvecs[i].tolist(),
                'distance': float(distance),
                'reproj_error': float(mean_error),
                'corner_errors': corner_errors
            }
            all_detections.append(detection_info)

            # 打印检测结果（每10帧打印一次）
            if idx % 10 == 0:
                print(f"\n{frame_name}: Marker ID={marker_id}")
                print(f"  Position: x={tvecs[i][0][0]:.3f}, y={tvecs[i][0][1]:.3f}, z={tvecs[i][0][2]:.3f}")
                print(f"  Distance: {distance:.3f} m")
                print(f"  Reprojection error: {mean_error:.2f} pixels")

    # 打印统计信息
    print("\n" + "="*80)
    print("DETECTION STATISTICS")
    print("="*80)
    print(f"Total images processed: {len(image_files)}")
    print(f"Images with detections: {detected_images}")
    print(f"Detection rate: {detected_images/len(image_files)*100:.1f}%")
    print(f"Total marker detections: {total_markers}")
    print(f"Unique marker IDs: {len(marker_ids_seen)}")
    print(f"Marker IDs: {sorted(marker_ids_seen)}")

    if all_detections:
        # 统计重投影误差
        reproj_errors = [d['reproj_error'] for d in all_detections]
        print(f"\nReprojection error statistics:")
        print(f"  Mean: {np.mean(reproj_errors):.2f} pixels")
        print(f"  Std:  {np.std(reproj_errors):.2f} pixels")
        print(f"  Min:  {np.min(reproj_errors):.2f} pixels")
        print(f"  Max:  {np.max(reproj_errors):.2f} pixels")

        # 统计距离
        distances = [d['distance'] for d in all_detections]
        print(f"\nDistance statistics:")
        print(f"  Mean: {np.mean(distances):.3f} m")
        print(f"  Std:  {np.std(distances):.3f} m")
        print(f"  Min:  {np.min(distances):.3f} m")
        print(f"  Max:  {np.max(distances):.3f} m")

        # 按 marker ID 分组统计
        print(f"\nPer-marker statistics:")
        for marker_id in sorted(marker_ids_seen):
            marker_dets = [d for d in all_detections if d['marker_id'] == marker_id]
            marker_errors = [d['reproj_error'] for d in marker_dets]
            marker_dists = [d['distance'] for d in marker_dets]
            print(f"  Marker {marker_id}:")
            print(f"    Detections: {len(marker_dets)}")
            print(f"    Mean reprojection error: {np.mean(marker_errors):.2f} pixels")
            print(f"    Distance range: {np.min(marker_dists):.2f} - {np.max(marker_dists):.2f} m")

    return all_detections

def visualize_detection(image_path, camera_yaml, marker_size=0.123):
    """可视化单张图像的检测结果"""
    camera_matrix, dist_coeffs = load_camera_params(camera_yaml)
    image = cv2.imread(image_path)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

    if ids is not None and len(ids) > 0:
        # 绘制检测结果
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # 估计位姿并绘制坐标轴
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            cv2.drawFrameAxes(image, camera_matrix, dist_coeffs,
                            rvecs[i], tvecs[i], marker_size * 0.5)

        print(f"Detected {len(ids)} markers: {ids.flatten().tolist()}")
    else:
        print("No markers detected")

    # 保存结果
    output_path = image_path.replace('.jpg', '_detected.jpg')
    cv2.imwrite(output_path, image)
    print(f"Saved detection result to: {output_path}")

    return image

if __name__ == "__main__":
    data_dir = "/home/hhy/2004ros/data/4_video_office_large"
    camera_yaml = os.path.join(data_dir, "camera.yml")

    # 运行检测测试
    detections = test_aruco_detection(data_dir, camera_yaml, marker_size=0.123)

    # 可视化第一张和最后一张图像
    image_files = sorted(glob.glob(os.path.join(data_dir, "image-*.jpg")))
    if image_files:
        print("\n" + "="*80)
        print("Visualizing first image...")
        visualize_detection(image_files[0], camera_yaml)

        print("\nVisualizing last image...")
        visualize_detection(image_files[-1], camera_yaml)
