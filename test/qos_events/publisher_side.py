"""Publisher side for TRANSIENT_LOCAL above depth 1, MANUAL_BY_TOPIC and
incompatible QoS."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks
from std_msgs.msg import String

rclpy.init()
node = Node('b_pub')

# 1) Publish five messages at TRANSIENT_LOCAL depth 5, then wait.
tl_qos = QoSProfile(depth=5,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE)
tl_pub = node.create_publisher(String, '/b_transient', tl_qos)
for i in range(5):
    tl_pub.publish(String(data=f'history-{i}'))
    time.sleep(0.05)

# 2) MANUAL_BY_TOPIC with a 1 s lease: write for two seconds, then stop.
mb_qos = QoSProfile(depth=10,
                    liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
                    liveliness_lease_duration=Duration(seconds=1))
lost = []
mb_pub = node.create_publisher(
    String, '/b_manual', mb_qos,
    event_callbacks=PublisherEventCallbacks(liveliness=lambda e: lost.append(e)))

# 3) A BEST_EFFORT publisher; the subscriber asks for RELIABLE, so incompatible.
be_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
offered = []
be_pub = node.create_publisher(
    String, '/b_incompatible', be_qos,
    event_callbacks=PublisherEventCallbacks(incompatible_qos=lambda e: offered.append(e)))

end = time.time() + 2.0
while time.time() < end:
    mb_pub.publish(String(data='alive'))
    be_pub.publish(String(data='x'))
    rclpy.spin_once(node, timeout_sec=0.1)

# Stay silent and let the lease expire.
end = time.time() + 5.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f'[pub] LIVELINESS_LOST fired {len(lost)} times, '
      f'total_count={lost[-1].total_count if lost else 0}')
print(f'[pub] OFFERED_QOS_INCOMPATIBLE fired {len(offered)} times, '
      f'policy={offered[-1].last_policy_kind if offered else None}')
ok = len(lost) > 0 and len(offered) > 0
print('=== PUB RESULT:', 'SUCCESS' if ok else 'FAILURE', '===')
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if ok else 1)
