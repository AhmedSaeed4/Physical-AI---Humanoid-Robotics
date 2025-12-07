---
sidebar_position: 1
title: "Chapter 7: Sensors & VSLAM"
---

# Chapter 7: Sensors & VSLAM

## Overview

This chapter covers the integration of sensors for humanoid robots and the implementation of Visual Simultaneous Localization and Mapping (VSLAM) systems. We'll explore how to create robust perception systems that enable robots to understand and navigate their environment.

## Learning Objectives

After completing this chapter, you will be able to:
- Integrate various sensor types for humanoid robots
- Implement VSLAM algorithms for mapping and localization
- Process and calibrate sensor data
- Create sensor fusion pipelines
- Optimize perception systems for real-time performance
- Handle sensor failures and uncertainties

## Sensor Types for Humanoid Robots

### 1. Vision Sensors

Vision sensors provide rich information about the environment:

#### RGB Cameras
- **Purpose**: Visual perception, object recognition, navigation
- **Resolution**: Typically 640x480 to 1920x1080
- **Frame Rate**: 15-60 FPS for real-time applications
- **Mounting**: Head, torso, or specialized positions

#### Stereo Cameras
- **Purpose**: Depth estimation, 3D reconstruction
- **Baseline**: Distance between cameras affects depth range
- **Algorithms**: Block matching, semi-global matching (SGM)

#### RGB-D Cameras
- **Purpose**: Dense depth information, object detection
- **Examples**: Intel RealSense, Microsoft Kinect, Orbbec Astra
- **Range**: 0.3m to 10m depending on model

### 2. Range Sensors

#### LIDAR
- **Purpose**: Accurate distance measurements, mapping
- **Types**: 2D (single plane) and 3D (multiple planes)
- **Accuracy**: Millimeter-level precision
- **Range**: 0.1m to 100m+ depending on model

#### Ultrasonic Sensors
- **Purpose**: Short-range obstacle detection
- **Range**: 0.02m to 4m
- **Frequency**: 20-200 kHz

### 3. Inertial Sensors

#### IMU (Inertial Measurement Unit)
- **Components**: Accelerometer, gyroscope, magnetometer
- **Purpose**: Orientation, motion, and position estimation
- **Update Rate**: 100-1000 Hz
- **Integration**: For dead reckoning and sensor fusion

#### Gyroscope
- **Purpose**: Angular velocity measurement
- **Drift**: Accumulates over time
- **Complementary**: Used with accelerometers

## Sensor Integration in ROS 2

### Camera Integration

```python
# camera_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraIntegrationNode(Node):
    """
    Node for camera integration and basic processing.
    """

    def __init__(self):
        """
        Initialize the camera integration node.
        """
        super().__init__('camera_integration')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.processed_image_publisher = self.create_publisher(
            Image,
            'camera/image_processed',
            10
        )

        # Camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info("Camera integration node initialized")

    def image_callback(self, msg: Image):
        """
        Process incoming camera images.

        Args:
            msg: Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply camera calibration if available
            if self.camera_matrix is not None and self.distortion_coeffs is not None:
                cv_image = cv2.undistort(
                    cv_image,
                    self.camera_matrix,
                    self.distortion_coeffs
                )

            # Perform basic image processing
            processed_image = self.process_image(cv_image)

            # Publish processed image
            processed_msg = self.cv_bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            processed_msg.header = msg.header
            self.processed_image_publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Process camera calibration information.

        Args:
            msg: CameraInfo message with calibration data
        """
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.image_width = msg.width
            self.image_height = msg.height

            self.get_logger().info(f"Updated camera calibration: {self.image_width}x{self.image_height}")

        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {e}")

    def process_image(self, image: np.ndarray) -> np.ndarray:
        """
        Perform basic image processing.

        Args:
            image: Input image

        Returns:
            Processed image
        """
        # Example: Convert to grayscale and detect edges
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Combine original and processed
        result = image.copy()
        result[:, :, 0] = np.where(edges > 0, 255, result[:, :, 0])  # Highlight edges in blue

        return result


def main(args=None):
    """
    Main function to run the camera integration node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    camera_node = CameraIntegrationNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        print("Camera integration node interrupted")
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### LIDAR Integration

```python
# lidar_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import KDTree


class LidarIntegrationNode(Node):
    """
    Node for LIDAR integration and point cloud processing.
    """

    def __init__(self):
        """
        Initialize the LIDAR integration node.
        """
        super().__init__('lidar_integration')

        # Create subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publishers
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            'pointcloud',
            10
        )

        self.obstacle_marker_publisher = self.create_publisher(
            MarkerArray,
            'obstacles',
            10
        )

        # LIDAR parameters
        self.range_min = 0.1
        self.range_max = 10.0
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = 0.0174533  # 1 degree

        # Obstacle detection parameters
        self.obstacle_threshold = 0.5  # meters
        self.min_cluster_size = 3  # minimum points for obstacle

        self.get_logger().info("LIDAR integration node initialized")

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming LIDAR scan data.

        Args:
            msg: LaserScan message
        """
        try:
            # Update parameters from message
            self.range_min = msg.range_min
            self.range_max = msg.range_max
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.angle_increment = msg.angle_increment

            # Convert scan to point cloud
            pointcloud = self.scan_to_pointcloud(msg)

            # Detect obstacles
            obstacles = self.detect_obstacles(pointcloud)

            # Publish obstacle markers
            self.publish_obstacle_markers(obstacles, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error processing scan: {e}")

    def scan_to_pointcloud(self, scan_msg: LaserScan) -> np.ndarray:
        """
        Convert LaserScan to point cloud.

        Args:
            scan_msg: LaserScan message

        Returns:
            Point cloud as numpy array [x, y, z]
        """
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(
            scan_msg.angle_min,
            scan_msg.angle_max,
            scan_msg.angle_increment
        )

        # Filter valid ranges
        valid_indices = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Convert to Cartesian coordinates
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)

        return np.column_stack((x, y, z))

    def detect_obstacles(self, pointcloud: np.ndarray) -> list:
        """
        Detect obstacles in point cloud.

        Args:
            pointcloud: Point cloud data

        Returns:
            List of obstacle centers and sizes
        """
        if len(pointcloud) == 0:
            return []

        # Build KD-tree for efficient neighbor search
        tree = KDTree(pointcloud[:, :2])  # Use only x,y for 2D clustering

        obstacles = []
        visited = set()

        for i, point in enumerate(pointcloud):
            if i in visited:
                continue

            # Find neighbors within threshold
            neighbors = tree.query_ball_point(point[:2], self.obstacle_threshold)

            if len(neighbors) >= self.min_cluster_size:
                # Calculate cluster center and size
                cluster_points = pointcloud[neighbors]
                center = np.mean(cluster_points, axis=0)
                size = np.std(cluster_points, axis=0)

                obstacles.append({
                    'center': center,
                    'size': size,
                    'points': len(neighbors)
                })

                # Mark all points in cluster as visited
                visited.update(neighbors)

        return obstacles

    def publish_obstacle_markers(self, obstacles: list, header):
        """
        Publish obstacle markers for visualization.

        Args:
            obstacles: List of detected obstacles
            header: Message header
        """
        marker_array = MarkerArray()

        for i, obstacle in enumerate(obstacles):
            # Create marker for obstacle
            marker = Marker()
            marker.header = header
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = float(obstacle['center'][0])
            marker.pose.position.y = float(obstacle['center'][1])
            marker.pose.position.z = float(obstacle['center'][2])
            marker.pose.orientation.w = 1.0

            # Set size (scale based on obstacle size)
            marker.scale.x = float(obstacle['size'][0] * 2.0)  # diameter
            marker.scale.y = float(obstacle['size'][1] * 2.0)
            marker.scale.z = 0.5  # fixed height

            # Set color (red for obstacles)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # alpha

            marker_array.markers.append(marker)

        self.obstacle_marker_publisher.publish(marker_array)


def main(args=None):
    """
    Main function to run the LIDAR integration node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    lidar_node = LidarIntegrationNode()

    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        print("LIDAR integration node interrupted")
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Visual SLAM Implementation

### ORB-SLAM Integration

```python
# orbslam_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Optional


class ORBSLAMNode(Node):
    """
    Node for ORB-SLAM integration with ROS 2.
    """

    def __init__(self):
        """
        Initialize the ORB-SLAM node.
        """
        super().__init__('orb_slam')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.odometry_publisher = self.create_publisher(
            Odometry,
            'orb_slam/odometry',
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'orb_slam/pose',
            10
        )

        # SLAM system state
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.last_image = None
        self.tracking_active = False
        self.map_points = []
        self.keyframes = []

        # ORB-SLAM parameters
        self.orb_features = 1000
        self.scale_factor = 1.2
        self.levels = 8
        self.edge_threshold = 19
        self.wta_k = 2
        self.score_type = cv2.ORB_HARRIS_SCORE
        self.patch_size = 31
        self.fast_threshold = 20

        # Initialize ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=self.orb_features,
            scaleFactor=self.scale_factor,
            nlevels=self.levels,
            edgeThreshold=self.edge_threshold,
            WTA_K=self.wta_k,
            scoreType=self.score_type,
            patchSize=self.patch_size,
            fastThreshold=self.fast_threshold
        )

        self.get_logger().info("ORB-SLAM node initialized")

    def image_callback(self, msg: Image):
        """
        Process incoming images for SLAM.

        Args:
            msg: Image message
        """
        try:
            # Convert to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Process with ORB-SLAM if camera parameters are available
            if self.camera_matrix is not None:
                pose = self.process_frame(cv_image, msg.header.stamp)

                if pose is not None:
                    self.publish_pose(pose, msg.header)

        except Exception as e:
            self.get_logger().error(f"Error processing image for SLAM: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Process camera calibration information.

        Args:
            msg: CameraInfo message
        """
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)

            self.get_logger().info(f"Camera calibration updated: {msg.width}x{msg.height}")

        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {e}")

    def process_frame(self, image: np.ndarray, timestamp) -> Optional[np.ndarray]:
        """
        Process a frame with ORB-SLAM algorithm.

        Args:
            image: Input image
            timestamp: Frame timestamp

        Returns:
            Camera pose or None if tracking lost
        """
        # Detect ORB features
        keypoints = self.orb.detect(image, None)
        keypoints, descriptors = self.orb.compute(image, keypoints)

        if descriptors is None:
            return None

        # Initialize if this is the first frame
        if not self.keyframes:
            # Create first keyframe
            first_keyframe = {
                'image': image,
                'keypoints': keypoints,
                'descriptors': descriptors,
                'pose': np.eye(4),  # Identity pose
                'timestamp': timestamp
            }
            self.keyframes.append(first_keyframe)
            self.tracking_active = True
            return first_keyframe['pose']

        # Track features with previous keyframe
        if self.tracking_active and len(self.keyframes) > 0:
            prev_keyframe = self.keyframes[-1]

            # Match features between current and previous frame
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(prev_keyframe['descriptors'], descriptors)

            if len(matches) >= 10:  # Minimum matches for reliable pose
                # Sort matches by distance
                matches = sorted(matches, key=lambda x: x.distance)

                # Extract matched points
                prev_pts = np.float32([prev_keyframe['keypoints'][m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
                curr_pts = np.float32([keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

                # Estimate pose using essential matrix
                E, mask = cv2.findEssentialMat(
                    prev_pts, curr_pts,
                    self.camera_matrix,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )

                if E is not None:
                    # Recover pose
                    _, R, t, mask = cv2.recoverPose(E, prev_pts, curr_pts, self.camera_matrix)

                    # Create transformation matrix
                    T = np.eye(4)
                    T[:3, :3] = R
                    T[:3, 3] = t.flatten()

                    # Update current pose relative to first frame
                    current_pose = prev_keyframe['pose'] @ T

                    # Add as new keyframe if significantly different
                    if self.is_significant_motion(T):
                        new_keyframe = {
                            'image': image,
                            'keypoints': keypoints,
                            'descriptors': descriptors,
                            'pose': current_pose,
                            'timestamp': timestamp
                        }
                        self.keyframes.append(new_keyframe)

                    return current_pose

        return None

    def is_significant_motion(self, transform: np.ndarray, threshold: float = 0.1) -> bool:
        """
        Check if motion is significant enough to create a new keyframe.

        Args:
            transform: Transformation matrix
            threshold: Motion threshold

        Returns:
            True if motion is significant
        """
        translation = np.linalg.norm(transform[:3, 3])
        rotation = np.arccos(np.clip((np.trace(transform[:3, :3]) - 1) / 2, -1, 1))

        return translation > threshold or rotation > threshold

    def publish_pose(self, pose: np.ndarray, header):
        """
        Publish the estimated camera pose.

        Args:
            pose: 4x4 transformation matrix
            header: Message header
        """
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.child_frame_id = 'camera_frame'

        # Extract position
        odom_msg.pose.pose.position.x = float(pose[0, 3])
        odom_msg.pose.pose.position.y = float(pose[1, 3])
        odom_msg.pose.pose.position.z = float(pose[2, 3])

        # Extract orientation (convert rotation matrix to quaternion)
        R = pose[:3, :3]
        qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)

        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Publish odometry
        self.odometry_publisher.publish(odom_msg)

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose = odom_msg.pose.pose

        # Publish pose
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    """
    Main function to run the ORB-SLAM node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    slam_node = ORBSLAMNode()

    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        print("ORB-SLAM node interrupted")
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## NVIDIA Isaac ROS Integration

### Isaac ROS Perception Pipeline

```python
# isaac_ros_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import json


class IsaacROSPipelineNode(Node):
    """
    Node for NVIDIA Isaac ROS perception pipeline integration.
    """

    def __init__(self):
        """
        Initialize the Isaac ROS pipeline node.
        """
        super().__init__('isaac_ros_pipeline')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create subscriptions for Isaac ROS sensors
        self.rgb_image_subscription = self.create_subscription(
            Image,
            'camera/rgb/image_rect_color',
            self.rgb_image_callback,
            10
        )

        self.depth_image_subscription = self.create_subscription(
            Image,
            'camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Create publishers
        self.perception_result_publisher = self.create_publisher(
            String,
            'perception/results',
            10
        )

        # Isaac ROS perception parameters
        self.depth_scale = 0.001  # Default for most depth cameras
        self.confidence_threshold = 0.7
        self.detection_iou_threshold = 0.5

        # Perception state
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.perception_results = []

        self.get_logger().info("Isaac ROS perception pipeline initialized")

    def rgb_image_callback(self, msg: Image):
        """
        Process RGB image from Isaac ROS camera.

        Args:
            msg: RGB image message
        """
        try:
            # Convert ROS Image to OpenCV image
            self.latest_rgb_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image if we also have depth and camera info
            if self.latest_depth_image is not None and self.camera_intrinsics is not None:
                results = self.process_perception_pipeline(
                    self.latest_rgb_image,
                    self.latest_depth_image,
                    self.camera_intrinsics
                )

                if results:
                    self.publish_perception_results(results)

        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")

    def depth_image_callback(self, msg: Image):
        """
        Process depth image from Isaac ROS camera.

        Args:
            msg: Depth image message
        """
        try:
            # Convert ROS Image to OpenCV image (assuming 16-bit depth)
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert to float32 and scale if needed
            if depth_image.dtype == np.uint16:
                depth_image = depth_image.astype(np.float32) * self.depth_scale

            self.latest_depth_image = depth_image

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """
        Process camera calibration information.

        Args:
            msg: CameraInfo message
        """
        try:
            self.camera_intrinsics = {
                'fx': msg.k[0],  # Focal length x
                'fy': msg.k[4],  # Focal length y
                'cx': msg.k[2],  # Principal point x
                'cy': msg.k[5],  # Principal point y
                'width': msg.width,
                'height': msg.height
            }

            self.get_logger().info(f"Camera intrinsics updated: {msg.width}x{msg.height}")

        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {e}")

    def imu_callback(self, msg: Imu):
        """
        Process IMU data for sensor fusion.

        Args:
            msg: IMU message
        """
        try:
            # Store IMU data for sensor fusion
            imu_data = {
                'linear_acceleration': {
                    'x': msg.linear_acceleration.x,
                    'y': msg.linear_acceleration.y,
                    'z': msg.linear_acceleration.z
                },
                'angular_velocity': {
                    'x': msg.angular_velocity.x,
                    'y': msg.angular_velocity.y,
                    'z': msg.angular_velocity.z
                },
                'orientation': {
                    'x': msg.orientation.x,
                    'y': msg.orientation.y,
                    'z': msg.orientation.z,
                    'w': msg.orientation.w
                },
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            }

            # Use IMU data for sensor fusion with visual odometry
            self.fuse_imu_visual_data(imu_data)

        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")

    def process_perception_pipeline(self, rgb_image: np.ndarray, depth_image: np.ndarray,
                                   camera_intrinsics: dict) -> list:
        """
        Process the full perception pipeline.

        Args:
            rgb_image: RGB image
            depth_image: Depth image
            camera_intrinsics: Camera intrinsic parameters

        Returns:
            List of perception results
        """
        results = []

        # 1. Object detection (simulated - in real implementation would use Isaac ROS DNN nodes)
        detections = self.simulate_object_detection(rgb_image)

        # 2. Depth estimation for detected objects
        for detection in detections:
            if detection['confidence'] > self.confidence_threshold:
                # Get depth at object center
                center_x = int(detection['bbox'][0] + detection['bbox'][2] / 2)
                center_y = int(detection['bbox'][1] + detection['bbox'][3] / 2)

                # Ensure coordinates are within image bounds
                center_x = max(0, min(center_x, depth_image.shape[1] - 1))
                center_y = max(0, min(center_y, depth_image.shape[0] - 1))

                depth = depth_image[center_y, center_x]

                # Convert pixel coordinates to 3D world coordinates
                if depth > 0:  # Valid depth
                    world_x = (center_x - camera_intrinsics['cx']) * depth / camera_intrinsics['fx']
                    world_y = (center_y - camera_intrinsics['cy']) * depth / camera_intrinsics['fy']
                    world_z = depth

                    detection['position_3d'] = {
                        'x': world_x,
                        'y': world_y,
                        'z': world_z
                    }

                    results.append(detection)

        return results

    def simulate_object_detection(self, image: np.ndarray) -> list:
        """
        Simulate object detection (in real implementation, this would use Isaac ROS DNN nodes).

        Args:
            image: Input image

        Returns:
            List of simulated detections
        """
        # In a real implementation, this would interface with Isaac ROS DNN detection nodes
        # For simulation, we'll create some fake detections

        # Simulate detection of a red mug
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2

        return [{
            'class': 'mug',
            'confidence': 0.85,
            'bbox': [center_x - 30, center_y - 40, 60, 80],  # [x, y, width, height]
            'color': 'red',
            'position_2d': {'x': center_x, 'y': center_y}
        }]

    def publish_perception_results(self, results: list):
        """
        Publish perception results.

        Args:
            results: List of perception results
        """
        try:
            result_msg = String()
            result_msg.data = json.dumps({
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'results': results
            })
            self.perception_result_publisher.publish(result_msg)

            self.get_logger().info(f"Published {len(results)} perception results")

        except Exception as e:
            self.get_logger().error(f"Error publishing perception results: {e}")

    def fuse_imu_visual_data(self, imu_data: dict):
        """
        Fuse IMU and visual data for robust state estimation.

        Args:
            imu_data: IMU measurement data
        """
        # In a real implementation, this would implement sensor fusion
        # algorithms like Extended Kalman Filter or complementary filter
        pass


def main(args=None):
    """
    Main function to run the Isaac ROS perception pipeline.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    perception_node = IsaacROSPipelineNode()

    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        print("Isaac ROS perception pipeline interrupted")
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Sensor Calibration

### Camera Calibration Pipeline

```python
# camera_calibration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool


class CameraCalibrationNode(Node):
    """
    Node for camera calibration using chessboard pattern.
    """

    def __init__(self):
        """
        Initialize the camera calibration node.
        """
        super().__init__('camera_calibration')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('calibration_pattern_size', [9, 6])
        self.declare_parameter('square_size', 0.025)  # 2.5cm squares
        self.declare_parameter('num_images_needed', 20)
        self.declare_parameter('calibration_flags', 0)

        # Get parameter values
        pattern_size = self.get_parameter('calibration_pattern_size').value
        self.pattern_size = (pattern_size[0], pattern_size[1])
        self.square_size = self.get_parameter('square_size').value
        self.num_images_needed = self.get_parameter('num_images_needed').value
        self.calibration_flags = self.get_parameter('calibration_flags').value

        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publishers
        self.calibration_status_publisher = self.create_publisher(
            String,
            'calibration/status',
            10
        )

        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            'camera/camera_info',
            10
        )

        self.calibration_complete_publisher = self.create_publisher(
            Bool,
            'calibration/complete',
            10
        )

        # Calibration state
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.obj_points = []  # 3D points in real world space
        self.img_points = []  # 2D points in image plane
        self.obj_p = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.obj_p[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.square_size

        self.calibration_images_collected = 0
        self.calibration_completed = False
        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info("Camera calibration node initialized")

    def image_callback(self, msg: Image):
        """
        Process incoming images for calibration.

        Args:
            msg: Image message
        """
        if self.calibration_completed:
            return  # Calibration already done

        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Find chessboard corners
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(
                gray, self.pattern_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if ret:
                # Refine corner locations
                refined_corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), self.criteria
                )

                # Add to calibration data if corners are sufficiently different from previous ones
                if self.should_add_calibration_image(refined_corners):
                    self.obj_points.append(self.obj_p)
                    self.img_points.append(refined_corners)
                    self.calibration_images_collected += 1

                    # Draw and display corners
                    cv2.drawChessboardCorners(cv_image, self.pattern_size, refined_corners, ret)

                    self.get_logger().info(f"Collected {self.calibration_images_collected}/{self.num_images_needed} calibration images")

                    # Check if we have enough images for calibration
                    if self.calibration_images_collected >= self.num_images_needed:
                        self.perform_calibration()
                        self.publish_calibration_results()

        except Exception as e:
            self.get_logger().error(f"Error processing calibration image: {e}")

    def should_add_calibration_image(self, new_corners: np.ndarray, min_distance: float = 0.1) -> bool:
        """
        Check if the new corners are sufficiently different from previous ones.

        Args:
            new_corners: New corner locations
            min_distance: Minimum distance threshold

        Returns:
            True if image should be added to calibration set
        """
        if not self.img_points:
            return True

        # Calculate average distance between new corners and previous corners
        for prev_corners in self.img_points:
            avg_distance = np.mean(np.linalg.norm(new_corners - prev_corners, axis=2))
            if avg_distance < min_distance:
                return False  # Too similar to previous image

        return True

    def perform_calibration(self):
        """
        Perform camera calibration using collected images.
        """
        try:
            if len(self.obj_points) < self.num_images_needed:
                self.get_logger().warn(f"Not enough calibration images: {len(self.obj_points)}/{self.num_images_needed}")
                return

            # Perform calibration
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points,
                (640, 480), None, None
            )

            if ret:
                self.camera_matrix = camera_matrix
                self.dist_coeffs = dist_coeffs
                self.calibration_completed = True

                self.get_logger().info(f"Calibration completed successfully: Reprojection error = {ret:.4f}")
                self.get_logger().info(f"Camera matrix:\n{camera_matrix}")
                self.get_logger().info(f"Distortion coefficients: {dist_coeffs.flatten()}")

            else:
                self.get_logger().error("Calibration failed")

        except Exception as e:
            self.get_logger().error(f"Error during calibration: {e}")

    def publish_calibration_results(self):
        """
        Publish calibration results.
        """
        try:
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                # Create and publish CameraInfo message
                camera_info_msg = CameraInfo()
                camera_info_msg.header.stamp = self.get_clock().now().to_msg()
                camera_info_msg.header.frame_id = 'camera_frame'

                # Set camera matrix
                camera_info_msg.k = self.camera_matrix.flatten().tolist()

                # Set distortion coefficients
                camera_info_msg.d = self.dist_coeffs.flatten().tolist()

                # Set image dimensions
                camera_info_msg.width = 640
                camera_info_msg.height = 480

                # Set projection matrix
                camera_info_msg.p = [
                    self.camera_matrix[0, 0], 0.0, self.camera_matrix[0, 2], 0.0,
                    0.0, self.camera_matrix[1, 1], self.camera_matrix[1, 2], 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]

                self.camera_info_publisher.publish(camera_info_msg)

                # Publish completion status
                complete_msg = Bool()
                complete_msg.data = True
                self.calibration_complete_publisher.publish(complete_msg)

                # Publish status
                status_msg = String()
                status_msg.data = json.dumps({
                    'status': 'calibration_complete',
                    'timestamp': time.time(),
                    'reprojection_error': float(ret) if 'ret' in locals() else 0.0
                })
                self.calibration_status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing calibration results: {e}")


def main(args=None):
    """
    Main function to run the camera calibration node.

    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)

    calibration_node = CameraCalibrationNode()

    try:
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        print("Camera calibration node interrupted")
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()