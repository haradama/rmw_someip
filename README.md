# rmw_someip

A ROS 2 middleware (RMW) implementation that runs on **SOME/IP** instead of DDS,
built on [vsomeip](https://github.com/COVESA/vsomeip).

```bash
export RMW_IMPLEMENTATION=rmw_someip_cpp
ros2 run demo_nodes_cpp talker
```

Unmodified ROS 2 applications talk SOME/IP on the wire. No bridge node, no code
changes, no interface definitions to duplicate.

> **Research-grade software.** It has not been run in production, and it has no
> authentication or encryption. See [Limitations](#limitations) and
> [Security](#security) before using it on a network you do not control.

## Contents

- [Why](#why)
- [Status](#status)
- [Quick start](#quick-start)
- [How it works](#how-it-works)
- [Talking to non-ROS SOME/IP applications](#talking-to-non-ros-someip-applications)
- [Python](#python)
- [QoS](#qos)
- [Configuration](#configuration)
- [Limitations](#limitations)
- [Security](#security)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [License](#license)

## Why

Automotive platforms (AUTOSAR Classic / Adaptive) use SOME/IP as their standard
protocol. The usual way to connect ROS 2 to them is a bridge node that converts
messages, which becomes both a bottleneck and a single point of failure — and
the ROS graph itself never becomes SOME/IP native.

The `rmw` interface is transport agnostic even though it was designed around
DDS, as `rmw_zenoh` demonstrates. This implementation puts the whole ROS graph
on SOME/IP so that a ROS 2 node and an AUTOSAR ECU are peers on the same bus.

## Status

Passes the official ROS 2 conformance suites with no test skipped:

- `test_rmw_implementation`
- `test_quality_of_service`
- `test_communication` (rclcpp / rclpy, all pairs)
- `test_rclcpp`

The rclcpp/rclpy pairs of `test_communication` fail intermittently on a
shutdown race inside rclpy; see
[docs/design.md](docs/design.md#limitations-and-risks).

On top of those come unit tests for the library internals and a byte-level CDR
compatibility check against `rmw_fastrtps_cpp`. CI measures line coverage of
`rmw_someip_cpp/src/` on every run.

### Latency

Local one-way latency is higher than `rmw_fastrtps_cpp`'s at every payload
size. The gap is structural: vsomeip routes even local traffic through a routing
manager process, while Fast DDS hands it over in shared memory.

Measure it on your own hardware with `./tools/ci/in-docker.sh benchmark`, which
runs the same publisher and subscriber against both implementations.

## Quick start

### Requirements

- ROS 2 Jazzy (Ubuntu 24.04)
- vsomeip 3.7.4 — not packaged for Ubuntu, so it has to be built from source

The provided Docker image contains both, and Docker is the only thing the steps
below need on the host.

### Build

```bash
# ROS 2 Jazzy plus vsomeip; building vsomeip takes a while the first time
docker build -f docker/base.Dockerfile -t rmw_someip/base:jazzy .

# The RMW itself
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/ws rmw_someip/base:jazzy bash -lc '
  source /opt/ros/jazzy/setup.bash && cd /ws &&
  colcon build --packages-select rmw_someip_cpp'
```

### Run

A talker and a listener in one container:

```bash
docker run --rm -it -v "$PWD":/ws rmw_someip/base:jazzy bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_someip_cpp
  ros2 run demo_nodes_cpp talker &
  ros2 run demo_nodes_cpp listener'
```

```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
```

### Across containers or hosts

SOME/IP service discovery is multicast, and vsomeip does not start it until the
host has a multicast route. A Docker container has none by default, so each
container needs `NET_ADMIN` and one extra command at startup:

```bash
ip route add 224.0.0.0/4 dev eth0
```

[demo/compose.yaml](demo/compose.yaml) and
[test/pubsub_smoke/compose.yaml](test/pubsub_smoke/compose.yaml) are working
examples. On a host with several network interfaces, also set
`RMW_SOMEIP_UNICAST_ADDRESS` to the address SOME/IP should use.

### Without Docker

Install vsomeip 3.7.4 under a prefix CMake can find, with
`ENABLE_SIGNAL_HANDLING` left off (the default) so that vsomeip does not take
SIGINT and SIGTERM away from the application. Then build `rmw_someip_cpp` in a
colcon workspace like any other ROS 2 package.
[docker/base.Dockerfile](docker/base.Dockerfile) is the reference for the exact
steps and dependencies.

## How it works

ROS 2 needs things SOME/IP has no room for: arbitrary string topic names,
type names, QoS, node names. SOME/IP-SD carries only numeric IDs, a version and
a TTL.

**One SOME/IP service per ROS domain; one instance per participant.**
Event `0x8000` / eventgroup `0x0001` is reserved as a *graph channel*: every
participant publishes its own slice of the ROS graph there as a SOME/IP **field**,
so a participant that joins late still receives the current graph. Everything
ROS-specific — topic names, type names, type hashes, QoS, GIDs — travels on that
channel. SOME/IP-SD is left to do only what it is good at: announcing that a
participant exists and where to reach it.

Topics get event IDs `0x8001+` and eventgroups `0x0002+`; services get method
IDs `0x0001+`, assigned sequentially at runtime. The generated vsomeip JSON only
has to declare the participant's own service, instance and ports, so adding
topics never requires a configuration change.

Messages are serialized at runtime through the rosidl introspection
typesupport, for both C and C++, so any ROS message type works without
generating per-type code.

See [docs/design.md](docs/design.md) for the full design.

## Talking to non-ROS SOME/IP applications

There are two modes, chosen per topic.

**Native** (ROS ↔ ROS) is the default. Payloads are plain CDR, byte-compatible
with DDS implementations, so `rosbag2` and DDS bridges interoperate.

**Extern** (ROS ↔ non-ROS SOME/IP) is enabled per topic through a mapping file.
Service, instance and event IDs are pinned to what the peer expects, and
payloads are serialized per the AUTOSAR SOME/IP Transformer (big endian, packed,
no encapsulation header).

```yaml
# someip_mapping.yaml — RMW_SOMEIP_MAPPING_FILE points at this
externs:
  /vehicle/status:
    direction: subscribe          # the peer offers, ROS receives
    service_id: "0x1234"
    instance_id: "0x0001"
    event_id: "0x8001"
    eventgroup_id: "0x0001"
    type: rmw_someip_demo_msgs/msg/VehicleStatus
    serialization: someip_native
```

[demo/](demo/) runs a native vsomeip application with no ROS dependency and a
ROS 2 node with no SOME/IP dependency in separate containers, exchanging
messages in both directions. The ROS side is available in C++ and in Python.
[demo/README.md](demo/README.md) has the commands, the expected output and an
annotated packet capture.

## Python

rclpy needs nothing special. Native mode is just ROS 2:

```bash
export RMW_IMPLEMENTATION=rmw_someip_cpp
ros2 run demo_nodes_py talker      # and listener, in another shell
```

For extern mode, point `RMW_SOMEIP_MAPPING_FILE` at a mapping file and write an
ordinary node:

```python
import rclpy
from rclpy.node import Node
from rmw_someip_demo_msgs.msg import VehicleStatus

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(
            VehicleStatus, '/vehicle/status', self.on_status, 10)

    def on_status(self, msg):
        self.get_logger().info(f'speed={msg.speed_kph}')
```

Nothing in it mentions SOME/IP. See [demo/py_app/](demo/py_app/) for the version
that talks to the native SOME/IP application.

rclpy and rclcpp reach the RMW through different typesupports — the C
introspection and the C++ one — so they are exercised separately;
`test_communication` covers every rclcpp/rclpy pair.

## QoS

| ROS 2 QoS | Mapping |
|---|---|
| `RELIABLE` / `BEST_EFFORT` | TCP / UDP endpoint |
| `TRANSIENT_LOCAL`, depth ≤ 1 | SOME/IP field (latest value delivered on subscribe) |
| `TRANSIENT_LOCAL`, depth > 1 | Event plus a send history replayed to each new subscriber |
| `KEEP_LAST` / `KEEP_ALL` | Receive queue bound |
| `DEADLINE` | Watchdog over send / receive intervals |
| `LIFESPAN` | Expired messages dropped from the receive queue |
| `LIVELINESS` | `AUTOMATIC` from the graph channel; `MANUAL_BY_TOPIC` from a watchdog |

SOME/IP does not refuse to connect on incompatible QoS the way DDS does, so
compatibility is checked against the graph cache before subscribing.

`BEST_EFFORT` messages larger than one UDP datagram are segmented with
SOME/IP-TP.

## Configuration

By default nothing needs configuring: the RMW generates the vsomeip JSON for
each process. These environment variables adjust it.

| Variable | Default | Effect |
|---|---|---|
| `RMW_SOMEIP_MAPPING_FILE` | unset | Extern mode mapping file |
| `RMW_SOMEIP_CONFIG_FILE` | unset | Use this vsomeip JSON instead of generating one |
| `RMW_SOMEIP_UNICAST_ADDRESS` | first non-loopback IPv4 address | Pin the local address (needed with multiple NICs) |
| `RMW_SOMEIP_ROUTING_HOST` | unset | Application name of a dedicated vsomeip routing manager on this host. When unset, the first process to start becomes the routing manager |
| `RMW_SOMEIP_TP_EVENT_COUNT` | `256` | Number of event IDs declared for SOME/IP-TP; raise it for a process with more topics than that |
| `RMW_SOMEIP_LOG_LEVEL` | `warning` | vsomeip log level |
| `RMW_SOMEIP_SERVICE_ID_BASE` | `0xF000` | Service ID of ROS domain 0; domain *n* uses base + *n* |
| `RMW_SOMEIP_SD_PORT_BASE` | `30490` | Service discovery port of ROS domain 0; domain *n* uses base + 2*n* |

Each participant also takes one UDP port from 31000–31999 and one TCP port from
32000–32999.

## Limitations

- `MESSAGE_LOST` is the one QoS event not supported; it would need a sequence
  number on the wire and therefore a format change.
- Content-filtered topics, network flow endpoints and dynamic type support
  return `RMW_RET_UNSUPPORTED`.
- Extern mode covers publish/subscribe only. ROS services need native mode's
  graph channel, so a node that uses extern topics alone should disable its
  parameter services (see [demo/README.md](demo/README.md)).
- Two processes cannot publish the same extern topic, because its service and
  instance IDs are pinned.
- Nodes using different RMW implementations cannot talk to each other directly,
  as with any pair of RMWs on different transports.
- Only ROS 2 Jazzy with vsomeip 3.7.4 on Linux has been tested.
- Local latency is higher than with a shared-memory DDS transport
  (see [Latency](#latency)).

## Security

SOME/IP as used here has no authentication, no access control and no
encryption, and SROS 2 security enclaves are not implemented. Anyone who can
reach the network can join the ROS graph, read every topic and publish to any of
them. Run it only on a trusted, isolated network.

The RMW writes its generated vsomeip configuration and its port lock files to
`/tmp` under predictable names, so it should not be used on a host shared with
untrusted local users.

## Testing

```bash
docker build -f docker/test.Dockerfile -t rmw_someip/test:jazzy .

./tools/ci/in-docker.sh lint          # ament_lint
./tools/ci/in-docker.sh unit          # unit tests + CDR byte compatibility
./tools/ci/in-docker.sh conformance   # official suites + QoS / loaned messages
./tools/ci/in-docker.sh coverage      # line coverage
./tools/ci/in-docker.sh benchmark     # latency against other RMW implementations
```

CI runs the same scripts in the same image, so there is nothing that passes
locally but fails there. The benchmark is the exception: it is too noisy for a
shared runner and is only run by hand. See [test/README.md](test/README.md) for
running each test on its own.

## Repository layout

| Path | Contents |
|---|---|
| [rmw_someip_cpp/](rmw_someip_cpp/) | The RMW implementation |
| [demo/](demo/) | Extern mode demo: a native SOME/IP application and a ROS 2 node, in C++ and Python |
| [test/](test/) | Unit tests, CDR compatibility, QoS events, loaned messages, smoke test and benchmark |
| [docs/design.md](docs/design.md) | The design |
| [docker/](docker/) | Base and test images |
| [tools/](tools/) | Build and CI scripts |

## License

Apache-2.0. See [LICENSE](LICENSE).

vsomeip is a separate project under the MPL-2.0; it is linked dynamically and
not distributed with this repository.
