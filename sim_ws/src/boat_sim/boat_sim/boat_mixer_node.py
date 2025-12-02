import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from mavros_msgs.msg import RCIn


def pwm_to_normalized(pwm, center=1500.0, span=500.0):
    """
    Convert PWM in microseconds to a normalized value in [-1, 1].

    1000 us -> about -1
    1500 us -> 0
    2000 us -> about +1
    """
    if pwm <= 0.0:
        # zero or missing value
        return 0.0

    norm = (pwm - center) / span
    if norm > 1.0:
        norm = 1.0
    if norm < -1.0:
        norm = -1.0
    return norm


class BoatMixerNode(Node):
    def __init__(self):
        super().__init__("boat_mixer")

        # Parameters for which RC channels we use
        self.declare_parameter("throttle_channel", 3)  # default: channel 3
        self.declare_parameter("steering_channel", 1)  # default: channel 1

        self.throttle_channel = (
            self.get_parameter("throttle_channel")
            .get_parameter_value()
            .integer_value
        )
        self.steering_channel = (
            self.get_parameter("steering_channel")
            .get_parameter_value()
            .integer_value
        )

        # Publisher topics match the Ignition Thruster command topics
        self.left_pub = self.create_publisher(
            Float64,
            "/model/nd_boat/joint/left_prop_joint/cmd_thrust",
            10,
        )
        self.right_pub = self.create_publisher(
            Float64,
            "/model/nd_boat/joint/right_prop_joint/cmd_thrust",
            10,
        )

        # Subscribe to MAVROS RC outputs
        self.rc_sub = self.create_subscription(
            RCIn,
            "/mavros/rc/in",
            self.rc_callback,
            10,
        )

        self.get_logger().info(
            f"Boat mixer started. "
            f"Throttle ch: {self.throttle_channel}, Steering ch: {self.steering_channel}"
        )

        # Command range to match Thruster plugin config
        self.max_cmd = 500.0

    def rc_callback(self, msg: RCIn):
        channels = msg.channels

        # RCOut channels are 1 based in ArduPilot naming: ch1, ch2, ...
        thr_idx = self.throttle_channel - 1
        steer_idx = self.steering_channel - 1

        if len(channels) <= max(thr_idx, steer_idx):
            self.get_logger().warn("RCIn message has too few channels")
            return

        throttle_pwm = channels[thr_idx]
        steering_pwm = channels[steer_idx]

        throttle = pwm_to_normalized(throttle_pwm)
        steering = pwm_to_normalized(steering_pwm)

        # Differential thrust mix in [-1, 1]
        left = throttle + steering
        right = throttle - steering

        # Clip to [-1, 1]
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        # Scale to Thruster command range [-max_cmd, max_cmd]
        left_cmd = left * self.max_cmd
        right_cmd = right * self.max_cmd

        # Publish
        left_msg = Float64()
        left_msg.data = float(left_cmd)
        right_msg = Float64()
        right_msg.data = float(right_cmd)

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # Optional debug
        self.get_logger().debug(
            f"RC thr={throttle_pwm} steer={steering_pwm} "
            f"norm=({throttle:.2f}, {steering:.2f}) "
            f"cmd=({left_cmd:.1f}, {right_cmd:.1f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BoatMixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
