"""TRANSIENT_LOCAL history within one process.

From a remote publisher the history arrives either through a SOME/IP field
(depth 1) or a notify_one() replay (depth above 1). Neither applies within one
process, where nothing goes through SOME/IP. This checks the history is not
asymmetric: available from another process but not from the same one.
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

rclpy.init()
node = Node('local_transient')

for depth, expected in ((1, 1), (5, 5)):
    qos = QoSProfile(depth=depth,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)
    topic = f'/local_transient_{depth}'
    pub = node.create_publisher(String, topic, qos)
    for i in range(depth):
        pub.publish(String(data=f'{depth}-{i}'))
    time.sleep(0.3)

    got = []
    node.create_subscription(String, topic, lambda m, g=got: g.append(m.data), qos)
    end = time.time() + 3
    while time.time() < end and len(got) < expected:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'depth={depth}: late subscribe -> {len(got)} {got} (expected {expected})')
    assert len(got) == expected, f'no history at depth={depth}'

print('=== LOCAL TRANSIENT RESULT: SUCCESS ===')
node.destroy_node(); rclpy.shutdown()
