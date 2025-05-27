import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv2 import aruco

class aruco(Node):
    def __init__(self):
        super().__init__('aruco')

        # Define the real-world length of the marker’s side (in meters)
        self.markerLength = 0.14

        # Camera matrix and distortion coefficients for simulation
        self.K = np.array([[528.433723449707,   0, 320],
                           [  0, 528.433723449707, 240],
                           [  0,   0,   1]])
        self.D = np.zeros(5)

        self.aruco_position = np.array([[-1.5, -2.145, np.pi/2],  # ArUco 0
                                        [0.3, 0.0, np.pi],        # ArUco 1
                                        [1.0, 2.14, -np.pi/2]     # ArUco 2 
                                        ])
        # Set up ArUco dictionary and detector
        self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.parameters)


        # Camera subscriber
        self.camera_subscriber = self.create_subscription(
            Image,  # Message type
            '/camera',  # Topic name (adjust based on your simulation setup)
            self.camera_callback,  # Callback function
            10  # QoS (queue size)
        )
        # CvBridge for image conversion
        self.bridge = CvBridge()

        # Aruco_pos publisher
        self.aruco_pos = self.create_publisher(Pose, '/aruco_pos', 10)
        self.aruco_detect = self.create_publisher(Int32, '/aruco_detect', 10)

    def camera_callback(self, msg):
        """Callback function to process camera images."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert image to grayscale
            gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

            # Detect ArUco markers
            markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(gray)

            if len(markerCorners) > 0:
                for i in range(len(markerIds)):
                    marker_id = markerIds[i][0]  # Obtén el ID del marcador actual
                    #self.get_logger().info(f"Detected marker ID: {marker_id}")
                    
                    # Estimate pose of each marker
                    rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners[i], self.markerLength, self.K, self.D)

                    # Compute distance from camera to marker
                    distance = np.linalg.norm(tvec[0][0])

                    # Calculate marker center in image
                    corners = markerCorners[i].reshape((4, 2))
                    topLeft, topRight, bottomRight, bottomLeft = corners
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                    # Draw marker and its pose axis
                    for corners in markerCorners:
                        corners = corners.reshape((4, 2))
                        topLeft, topRight, bottomRight, bottomLeft = corners
                        topLeft = tuple(map(int, topLeft))
                        topRight = tuple(map(int, topRight))
                        bottomRight = tuple(map(int, bottomRight))
                        bottomLeft = tuple(map(int, bottomLeft))
                        cv.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                        cv.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                        cv.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                    cv.drawFrameAxes(cv_image, self.K, self.D, rvec, tvec, 0.01)

                    # Extraer los valores de traslación desde tvec
                    tvec_x = tvec[0][0][0]  # Traslación en X
                    tvec_y = tvec[0][0][1]  # Traslación en Y

                    # Calcular la posición del robot en el sistema global
                    x_camara = self.aruco_position[marker_id][0] + distance * np.cos(self.aruco_position[marker_id][2] + tvec_x)
                    y_camara = self.aruco_position[marker_id][1] + distance * np.sin(self.aruco_position[marker_id][2] + tvec_y)

                    offset = 0.1  # Desplazamiento de 10 cm hacia atrás
                    x_robot = x_camara + offset * np.cos(self.aruco_position[marker_id][2])
                    y_robot = y_camara + offset * np.sin(self.aruco_position[marker_id][2])

                    # Convertir rvec a matriz de rotación
                    rotation_matrix, _ = cv.Rodrigues(rvec)

                    # Extraer el ángulo yaw (rotación alrededor del eje Z)
                    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

                    # Crear mensaje Pose para publicar la posición del marcador
                    pose_msg = Pose()
                    pose_msg.position.x = x_robot
                    pose_msg.position.y = y_robot
                    pose_msg.orientation.z = yaw

                    if distance <= 1.0: # Si la distancia es menor o igual a 2 metros
                        # Publicar la posición del robot
                        self.aruco_pos.publish(pose_msg)
            else:
                # Crear mensaje Pose para publicar la posición del marcador
                pose_msg = Pose()
                pose_msg.position.x = 0.0
                pose_msg.position.y = 0.0
                pose_msg.orientation.z = 0.0

                # Publicar la posición del robot
                self.aruco_pos.publish(pose_msg)


            # Display the image with detected markers
            cv.imshow("Aruco Detection", cv_image)
            cv.waitKey(1)  # Required for OpenCV to process the image
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")


def main(args=None):
    rclpy.init(args=args)
    aruco_node = aruco()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()  # Close OpenCV windows when the node shuts down


if __name__ == '__main__':
    main()