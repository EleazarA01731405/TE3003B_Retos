import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv2 import aruco
import yaml
from yaml.loader import SafeLoader
import math
import os
import ament_index_python.packages
from geometry_msgs.msg import PoseStamped

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/processed',
            self.image_callback,
            10)
        
        self.image_pub = self.create_publisher(Image, '/aruco_image', 10)
        self.info_pub = self.create_publisher(Float32MultiArray, '/aruco_info', 10)

        # Load calibration from YAML
        package_share = ament_index_python.packages.get_package_share_directory('aruco_create')
        calib_path = os.path.join(package_share, 'calibration_matrix', 'calibration_matrix.yaml')
        with open(calib_path) as f:
            data = yaml.load(f, Loader=SafeLoader)
        self.K = np.array(data['camera_matrix'])
        self.D = np.array(data['dist_coeff'])

        # Marker size in meters (adjust as needed)
        self.markerLength = 0.045

        # ArUco dictionary and detector
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        # Do NOT use ArucoDetector for OpenCV < 4.7

        self.distance = None
        self.angle = None
        self.markerIds = None

        self.get_logger().info('ArucoDetectorNode started.')


    def rvec_to_yaw(self, rvec):
        # Convert rotation vector to rotation matrix
        R, _ = cv.Rodrigues(rvec)
        # Yaw is the rotation around the Y axis (for OpenCV's camera frame)
        # For a marker lying flat, yaw = atan2(R[1,0], R[0,0])
        # But for ArUco, the Z axis points out of the marker, so use atan2(R[0,2], R[2,2])
        yaw = math.atan2(R[0,2], R[2,2])
        return yaw

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

            # Detect ArUco markers
            markerCorners, self.markerIds, rejectedCandidates = aruco.detectMarkers(
                gray, self.dictionary, parameters=self.parameters)

            info_msg = Float32MultiArray()
            info_msg.data = []

            if self.markerIds is not None and len(markerCorners) > 0:
                aruco.drawDetectedMarkers(cv_image, markerCorners, self.markerIds)
                for i in range(len(self.markerIds)):
                    # Estimate pose of each marker
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                        markerCorners[i], self.markerLength, self.K, self.D)

                    # Compute distance from camera to marker
                    self.distance = float(np.linalg.norm(tvec[0][0]))

                    # Compute angle (yaw) in radians relative to camera center using rvec
                    self.angle = self.rvec_to_yaw(rvec[0][0])  # Now in radians

                    # Calculate marker center in image
                    corners = markerCorners[i].reshape((4, 2))
                    cX = int((corners[0][0] + corners[2][0]) / 2.0)
                    cY = int((corners[0][1] + corners[2][1]) / 2.0)

                    # Draw marker and its pose axis
                    for corners_draw in markerCorners:
                        corners_draw = corners_draw.reshape((4, 2))
                        topLeft, topRight, bottomRight, bottomLeft = corners_draw
                        topLeft = tuple(map(int, topLeft))
                        topRight = tuple(map(int, topRight))
                        bottomRight = tuple(map(int, bottomRight))
                        bottomLeft = tuple(map(int, bottomLeft))
                        cv.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                        cv.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                        cv.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                    cv.drawFrameAxes(cv_image, self.K, self.D, rvec, tvec, 0.03)

                    # Annotate distance and angle on image
                    cv.putText(cv_image, f"{self.distance:.2f}m, {self.angle:.1f}deg", (cX, cY - 15),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Add info: [id, distance, angle]
                    info_msg.data.extend([int(self.markerIds[i][0]), self.distance, self.angle])

                    self.return_actual_pose()

            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(img_msg)

            # Publish info (empty if no marker)
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def return_actual_pose(self):
        self.get_logger().info("Calculando pose...")
        if self.angle is not None and self.distance is not None:
            # Define your known ArUco marker positions (id: [x, y, yaw])
            aruco_positions = {
                0: [1.0, 2.0, 0.0],
                1: [2.5, 3.5, 1.57],
                2: [4.0, 1.0, 3.14],
                3: [1.0, 1.0, 1.57],
            }

            # Use the last detected marker id, distance, and angle
            if self.distance is not None and self.angle is not None and self.markerIds is not None and len(self.markerIds) > 0:
                marker_id = int(self.markerIds[0][0])
                if marker_id in aruco_positions:
                    marker_x, marker_y, marker_yaw = aruco_positions[marker_id]
                    # Calculate robot's pose relative to marker
                    theta = math.radians(self.angle) + marker_yaw
                    robot_x = marker_x - self.distance * np.sin(theta)
                    robot_y = marker_y - self.distance * np.cos(theta)
                    robot_yaw = (marker_yaw + math.radians(self.angle)) % (2 * math.pi)
                    self.get_logger().info(
                        f"Estimated robot position in world from ArUco {marker_id}: x={robot_x:.2f}, y={robot_y:.2f}, yaw={robot_yaw:.2f}"
                    )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()