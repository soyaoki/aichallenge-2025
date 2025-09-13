#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoware_auto_mapping_msgs.msg import HADMapBin
from autoware_map_msgs.msg import LaneletMapBin


class HADMapBinToLaneletMapBin(Node):
    def __init__(self):
        super().__init__("hadmapbin_to_laneletmapbin")

        # QoS設定（TRANSIENT_LOCAL にして過去のメッセージも受信可能にする）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscriber（HADMapBin型を受信）
        self.sub = self.create_subscription(
            HADMapBin,
            "/map/vector_map",
            self.callback,
            qos_profile
        )

        # Publisher（LaneletMapBin型を配信）
        self.pub = self.create_publisher(
            LaneletMapBin,
            "/map/lanelet_map_bin",
            qos_profile
        )

        self.get_logger().info("✅ HADMapBin → LaneletMapBin converter node started.")

    def callback(self, msg: HADMapBin):
        # self.get_logger().info(msg)
        # 変換処理
        converted = LaneletMapBin()
        converted.header = msg.header
        converted.version_map_format = msg.format_version
        converted.version_map = msg.map_version
        converted.data = msg.data

        # Publish
        self.pub.publish(converted)
        self.get_logger().info("🟢 Converted and published LaneletMapBin message.")


def main(args=None):
    rclpy.init(args=args)
    node = HADMapBinToLaneletMapBin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 Shutting down hadmapbin_to_laneletmapbin node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
