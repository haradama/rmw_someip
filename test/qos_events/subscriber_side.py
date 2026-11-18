"""Subscriber side, started after the publisher."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
from rclpy.event_handler import SubscriptionEventCallbacks
from std_msgs.msg import String

rclpy.init()
node = Node('b_sub')
history, liveliness_events, incompatible = [], [], []

# 1) Subscribe late at TRANSIENT_LOCAL depth 5; five history entries expected.
tl_qos = QoSProfile(depth=5,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE)
node.create_subscription(String, '/b_transient', lambda m: history.append(m.data), tl_qos)

# 2) MANUAL_BY_TOPIC lease=1s
mb_qos = QoSProfile(depth=10,
                    liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
                    liveliness_lease_duration=Duration(seconds=1))
node.create_subscription(
    String, '/b_manual', lambda m: None, mb_qos,
    event_callbacks=SubscriptionEventCallbacks(liveliness=lambda e: liveliness_events.append(e)))

# 3) Ask for RELIABLE against a BEST_EFFORT publisher, so incompatible.
re_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
node.create_subscription(
    String, '/b_incompatible', lambda m: None, re_qos,
    event_callbacks=SubscriptionEventCallbacks(incompatible_qos=lambda e: incompatible.append(e)))

end = time.time() + 8.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

print(f'[sub] TRANSIENT_LOCAL history received={len(history)} {history}')
print(f'[sub] LIVELINESS_CHANGED={[(e.alive_count, e.not_alive_count, e.alive_count_change, e.not_alive_count_change) for e in liveliness_events]}')
print(f'[sub] REQUESTED_QOS_INCOMPATIBLE fired {len(incompatible)} times, '
      f'policy={incompatible[-1].last_policy_kind if incompatible else None}')

ok = (len(history) == 5
      and any(e.not_alive_count == 1 for e in liveliness_events)
      and len(incompatible) > 0)
print('=== SUB RESULT:', 'SUCCESS' if ok else 'FAILURE', '===')
node.destroy_node(); rclpy.shutdown()
sys.exit(0 if ok else 1)
