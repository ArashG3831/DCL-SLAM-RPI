import rclpy
from sensor_msgs.msg import Imu


class ImuWebotsPlugin:
    def __init__(self):
        self.robot = None
        self.ros_node = None
        self.publisher = None
        self.gyro = None
        self.accelerometer = None
        self.ready = False
        self.reported_not_ready = False

    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros_node = rclpy.create_node("epuck_imu_webots_plugin")
        self.publisher = self.ros_node.create_publisher(Imu, "/imu/data_raw", 10)

        timestep = int(self.robot.getBasicTimeStep())

        self.gyro = self.robot.getDevice("d500_gyro")
        self.accelerometer = self.robot.getDevice("d500_accelerometer")

        if self.gyro is None:
            self.ros_node.get_logger().error("Webots device 'd500_gyro' was not found.")
            return

        if self.accelerometer is None:
            self.ros_node.get_logger().error("Webots device 'd500_accelerometer' was not found.")
            return

        self.gyro.enable(timestep)
        self.accelerometer.enable(timestep)

        self.ready = True
        self.ros_node.get_logger().info(
            "IMU Webots plugin started: publishing /imu/data_raw"
        )

    def step(self):
        if not self.ready:
            if self.ros_node is not None and not self.reported_not_ready:
                self.ros_node.get_logger().warn("IMU plugin step called before ready.")
                self.reported_not_ready = True
            return

        # No subscriptions here, but spin_once is harmless and keeps the ROS node healthy.
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        gyro = self.gyro.getValues()
        accel = self.accelerometer.getValues()

        msg = Imu()

        t = float(self.robot.getTime())
        sec = int(t)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((t - sec) * 1_000_000_000)

        msg.header.frame_id = "base_link"

        # 6-axis IMU: no absolute orientation.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])

        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])

        msg.angular_velocity_covariance[0] = 0.0025
        msg.angular_velocity_covariance[4] = 0.0025
        msg.angular_velocity_covariance[8] = 0.0025

        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

        self.publisher.publish(msg)
