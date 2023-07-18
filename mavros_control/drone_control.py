import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import Altitude, State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointPull, WaypointClear
from geometry_msgs.msg import PoseStamped


class OffboardController(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        self.current_state = State()

        # Subscribe to necessary topics
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.coordinate_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, 10)
        self.altitude_sub = self.create_subscription(Altitude, '/mavros/altitude', self.altitude_callback, 10)
        self.waypoint_sub = self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.waypoint_callback, 10)
        self.waypoint_reached_sub = self.create_subscription(WaypointReached, '/mavros/mission/reached', self.waypoint_reached_callback, 10)
        self.local_sub = self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.current_waypoint = WaypointList()
        self.reached_waypoint = WaypointReached()
        self.current_pose = PoseStamped()
        self.global_coord = NavSatFix()

    def state_callback(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.global_coord = msg

    def altitude_callback(self, msg):
        self.current_altitude = msg.relative

    def waypoint_callback(self, msg):
        self.current_waypoint = msg

    def waypoint_reached_callback(self, msg):
        self.reached_waypoint = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_arm(self, arm):
        # Create client for the "arming" service
        arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')

        request = CommandBool.Request()
        request.value = arm

        future = arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success and self.current_state.armed == arm:
            self.get_logger().info('Vehicle armed: {}'.format(arm))
            return True
        else:
            self.get_logger().warn('Failed to arm vehicle')
            return False

    def set_mode(self, mode):
        # Create client for the "set_mode" service
        set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

        request = SetMode.Request()
        request.custom_mode = mode

        future = set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent and self.current_state.mode == mode:
            self.get_logger().info('Mode set to: {}'.format(mode))
            return True
        else:
            self.get_logger().warn('Failed to set mode: {}'.format(mode))
            return False

    def send_waypoints(self, waypoints):
        # Create client for the "waypoint_push" service
        waypoint_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not waypoint_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint push service...')

        request = WaypointPush.Request()
        request.waypoints = waypoints

        future = waypoint_push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Waypoint push successful')
            return True
        else:
            self.get_logger().warn('Waypoint push failed')
            return False

    def clear_waypoints(self):
        # Create client for the "waypoint_clear" service
        waypoint_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        while not waypoint_clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint clear service...')

        request = WaypointClear.Request()

        future = waypoint_clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Waypoint clear successful')
            return True
        else:
            self.get_logger().warn('Waypoint clear failed')
            return False

    def create_waypoint(self, frame, command, is_current, p1, p2, p3, p4, x, y, z):
        waypoint = Waypoint()
        waypoint.frame = frame
        waypoint.command = command
        waypoint.is_current = is_current
        waypoint.autocontinue = True
        waypoint.param1 = float(p1)
        waypoint.param2 = float(p2)
        waypoint.param3 = float(p3)
        waypoint.param4 = float(p4)
        waypoint.x_lat = x
        waypoint.y_long = y
        waypoint.z_alt = z
        return waypoint

    def is_mission_completed(self):
        # Create client for the "waypoint_pull" service
        waypoint_pull_client = self.create_client(WaypointPull, '/mavros/mission/pull')
        while not waypoint_pull_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for waypoint pull service...')

        request = WaypointPull.Request()

        future = waypoint_pull_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            num_waypoints = future.result().wp_received - 1

            if self.reached_waypoint.wp_seq == num_waypoints and self.reached_waypoint.header.stamp.sec == self.current_state.header.stamp.sec:
                return True
            else:
                return False
        else:
            self.get_logger().warn('Failed to pull waypoints')
            return False

    def target_reached(self, current, target, acceptance):
        x_1, y_1, z_1 = current.pose.position.x, current.pose.position.y, current.pose.position.z
        x_2, y_2, z_2 = target.pose.position.x, target.pose.position.y, target.pose.position.z

        distance = math.sqrt((x_1 - x_2)**2 + (y_1 - y_2)**2 + (z_1 - z_2)**2)
        if distance <= acceptance:
            return True
        else:
            return False

    def send_to_coordinate(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        # rate = self.create_rate(20)
        while rclpy.ok() and not self.target_reached(self.current_pose, target_pose, 0.25):
            self.pose_pub.publish(target_pose)
            if self.current_state.mode != 'OFFBOARD':
                self.set_mode('OFFBOARD')
            # rate.sleep()

    def magical_shape(self):
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start_z = self.current_pose.pose.position.z
        scale = 20
        self.send_to_coordinate(start_x, start_y + scale*1, start_z)
        self.send_to_coordinate(start_x - scale*1, start_y + scale*1, start_z)
        self.send_to_coordinate(start_x - scale*1, start_y, start_z)
        self.send_to_coordinate(start_x + scale*1, start_y, start_z)
        self.send_to_coordinate(start_x + scale*1, start_y + scale*1, start_z)
        self.send_to_coordinate(start_x, start_y + scale*1, start_z)

    def run(self):
        self.get_logger().info("Waiting for FCU connection...")
        # Wait until the FCU is connected
        while not self.current_state.connected:
            rclpy.spin_once(self)

        self.get_logger().info("FCU connected")

        waypoints = [
            self.create_waypoint(3, 22, True, 0, 0, 0, float('nan'), 47.398290, 8.545723, 10.0),
            self.create_waypoint(3, 16, False, 0, 10, 0, float('nan'), 47.399029, 8.545045, 10.0)
        ]

        self.get_logger().info("Clearing Old Waypoints...")
        while not self.clear_waypoints():
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Sending New Waypoints...")
        while not self.send_waypoints(waypoints):
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Entering Mission Mode...")
        while not self.set_mode("AUTO.MISSION"):
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Arming...")
        while not self.set_arm(True):
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Waiting for mission to complete...")
        while not self.is_mission_completed():
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Waypoint mission complete...")

        self.magical_shape()

        waypoints = [
            self.create_waypoint(3, 16, False, 0, 10, 0, float('nan'), 47.397552, 8.545120, 10.0),
            self.create_waypoint(3, 21, False, 0, 0, 0, float('nan'), 47.398290, 8.545723, 0.0)
        ]

        self.get_logger().info("Clearing Old Waypoints...")
        while not self.clear_waypoints():
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Sending New Waypoints...")
        while not self.send_waypoints(waypoints):
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Entering Mission Mode...")
        while not self.set_mode("AUTO.MISSION"):
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Waiting for mission to complete...")
        while not self.is_mission_completed():
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Waypoint mission complete...")


def main(args=None):
    rclpy.init(args=args)
    controller = OffboardController()
    controller.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
