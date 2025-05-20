import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import cos, sin, pi


class PointGeneratorNode(Node):
    def __init__(self):
        super().__init__('point_generator_node')

        # Declare parameters
        self.declare_parameter('gen_type', 0)  # Default is type 0 (polygon generation)
        self.declare_parameter('n_points', 4)  # Default is 4 points (for polygons)
        self.declare_parameter('point', [0.0, 0.0])  # Default target point for type 1

        # Publisher for the generated points
        self.points_publisher = self.create_publisher(
            Float32MultiArray,  # Message type
            'generated_points',  # Topic name
            10  # QoS (queue size)
        )

        # Timer to periodically publish points
        self.timer = self.create_timer(1.0, self.publish_points)

        # Add a parameter callback to handle dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Node started
        self.get_logger().info('PointGeneratorNode started 🚀')

    def calculate_polygon_points(self, num_points):
        """Calculate points on a unit circle for a regular polygon."""
        points = []
        for i in range(num_points):
            angle = 2 * pi * i / num_points  # Angle for each point
            x = cos(angle)  # X-coordinate
            y = sin(angle)  # Y-coordinate
            points.append((x, y))
        points.append(points[0])  # Close the polygon by returning to the starting point
        return points

    def publish_points(self):
        """Publish the generated points based on the gen_type."""
        gen_type = self.get_parameter('gen_type').value

        if gen_type == 0:
            # Regular polygon generation
            num_points = self.get_parameter('n_points').value
            points = self.calculate_polygon_points(num_points)
            self.get_logger().info(f'Generated polygon with {num_points} points: {points}')
        elif gen_type == 1:
            # Single point generation
            point = self.get_parameter('point').value
            if len(point) != 2:
                self.get_logger().error("Invalid 'point' parameter. It must be a list of two values [x, y].")
                return
            points = [tuple(point)]  # Wrap the single point in a list
            self.get_logger().info(f'Generated single point: {points}')
        else:
            self.get_logger().error(f"Invalid gen_type: {gen_type}. Must be 0 or 1.")
            return

        # Convert points to a Float32MultiArray message
        msg = Float32MultiArray()
        for x, y in points:
            msg.data.extend([x, y])

        # Publish the points
        self.points_publisher.publish(msg)
        self.get_logger().info(f'Published points: {points}')

    def parameter_callback(self, params):
        """Callback function to handle parameter updates."""
        for param in params:
            if param.name == 'gen_type' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.get_logger().info(f"Updated gen_type to {param.value}")
            elif param.name == 'n_points' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.get_logger().info(f"Updated n_points to {param.value}")
            elif param.name == 'point' and param.type_ == rclpy.Parameter.Type.DOUBLE_ARRAY:
                self.get_logger().info(f"Updated point to {param.value}")

        # Trigger immediate re-execution of the logic after parameter change
        self.publish_points()

        return rclpy.parameter.ParameterEventCallbackResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    node = PointGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()