# path_visualizer_from_trajectory.py
import rclpy
from rclpy.node import Node

#from nav_msgs.msg import Path #\trajectory
from nav_msgs.msg import Odometry #\odom

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Int32
from builtin_interfaces.msg import Duration

class PathVisualizer(Node):

    def __init__(self):
        super().__init__('path_visualizer')

        self.publisher = self.create_publisher(Marker, 'path_marker', 10)
        #self.subscription = self.create_subscription( #trajectory
        #    Path,
        #    'trajectory',
        #    self.trajectory_callback,
        #    10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.color_subscription = self.create_subscription(
            Int32,
            'line_color',
            self.color_callback,
            10)

        #self.current_color_id = 0
        #self.segment_points = []  # Store as pairs of points (LINE_LIST)
        #self.segment_colors = []

        self.current_color_id = 0
        self.last_position = None
        self.segment_points = []
        self.segment_colors = []

        self.marker = Marker()
        self.init_marker()

    def init_marker(self):
        self.marker.header.frame_id = "odom"
        self.marker.ns = "path"
        self.marker.id = 0
        self.marker.type = Marker.LINE_LIST  # <-- Important change
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.005
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = Duration(sec=0)

    def color_callback(self, msg: Int32):
        self.get_logger().info(f"Received new color ID: {msg.data}")
        self.current_color_id = msg.data

    def get_color(self, color_id: int) -> ColorRGBA:
        if color_id == 0:  # yellow
            return ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        elif color_id == 1:  # red
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif color_id == 2:  # blue
            return ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        else:  # default gray
            return ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)

    # def trajectory_callback(self, msg: Path):
    #     poses = msg.poses
    #     if len(poses) < 2:
    #         return  # Not enough points for a segment

    #     # Only add new segments
    #     last_index = len(self.segment_points) // 2
    #     new_poses = poses[last_index + 1:]

    #     for i in range(1, len(new_poses)):
    #         start = new_poses[i - 1].pose.position
    #         end = new_poses[i].pose.position
    #         self.segment_points.append(start)
    #         self.segment_points.append(end)
    #         color = self.get_color(self.current_color_id)
    #         self.segment_colors.append(color)
    #         self.segment_colors.append(color)

    #     self.marker.points = self.segment_points
    #     self.marker.colors = self.segment_colors
    #     self.marker.header.stamp = self.get_clock().now().to_msg()
    #     self.publisher.publish(self.marker)
        
    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position

        if self.last_position is not None:
            self.segment_points.append(self.last_position)
            self.segment_points.append(position)

            color = self.get_color(self.current_color_id)
            self.segment_colors.append(color)
            self.segment_colors.append(color)

            self.marker.points = self.segment_points
            self.marker.colors = self.segment_colors
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.marker)

        self.last_position = position

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
