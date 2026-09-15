import signal

import rclpy
from my_epuck_interfaces.msg import PeerMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


class MapExporter(Node):
    """Export only this robot's original local SLAM map as source-aware evidence."""

    def __init__(self, **node_kwargs):
        super().__init__('map_exporter', **node_kwargs)
        self.declare_parameter('source_robot_id', '')
        self.declare_parameter('input_topic', 'map')
        self.declare_parameter('output_topic', '/cslam/local_map')
        self.declare_parameter('export_rate_hz', 1.0)

        self.source_robot_id = self.get_parameter('source_robot_id').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        export_rate = float(self.get_parameter('export_rate_hz').value)
        if not self.source_robot_id:
            raise ValueError('source_robot_id must not be empty')
        if export_rate <= 0.0:
            raise ValueError('export_rate_hz must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(PeerMap, output_topic, qos)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, input_topic, self.map_callback, qos
        )
        self.latest_map = None
        self.revision = 0
        self._finalized = False
        self.timer = self.create_timer(1.0 / export_rate, self.export_map)
        self.get_logger().info(
            f'Exporting local evidence only: {self.resolve_topic_name(input_topic)} '
            f'-> {self.resolve_topic_name(output_topic)} at {export_rate:.2f} Hz '
            f'as source {self.source_robot_id}'
        )

    def map_callback(self, message):
        self.latest_map = message

    def export_map(self):
        if self._finalized or self.latest_map is None or not self.context.ok():
            return
        self.revision += 1
        message = PeerMap()
        message.source_robot_id = self.source_robot_id
        message.revision = self.revision
        message.export_stamp = self.get_clock().now().to_msg()
        message.local_evidence_only = True
        message.occupancy_grid = self.latest_map
        if rclpy.ok():
            self.publisher.publish(message)

    def finalize(self):
        """Publish the buffered map once while the ROS context is valid."""
        if self._finalized:
            return False
        if self.context.ok() and rclpy.ok():
            self.export_map()
            self._finalized = True
            return True
        self._finalized = True
        return False


def shutdown_node(node, executor=None):
    """Finalize buffered evidence before ordered ROS entity teardown."""
    if node is not None:
        node.finalize()
    if executor is not None:
        executor.remove_node(node)
        executor.shutdown()
    if node is not None and node.context.ok():
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    shutdown_requested = {'value': False}
    previous_handlers = {}

    def request_shutdown(signum, frame):
        del signum, frame
        shutdown_requested['value'] = True
        if executor is not None:
            executor.wake()

    try:
        node = MapExporter()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_shutdown)
        while not shutdown_requested['value'] and rclpy.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        shutdown_node(node, executor)


if __name__ == '__main__':
    main()
