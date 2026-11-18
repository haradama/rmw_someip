# rmw_someip architecture

The design of an implementation that uses SOME/IP (vsomeip) as a ROS 2 RMW.

> **This document was revised after the Phase 0 measurements.** The first
> version assumed one SOME/IP service per topic; measurement showed that does
> not work. §5 replaces it, based on the results in §3.

## 1. Background and goal

ROS 2's `rmw` layer was designed around DDS, but the interface itself is
transport agnostic, as `rmw_zenoh` demonstrates.

In automotive, the AUTOSAR Classic and Adaptive platforms use SOME/IP as their
standard protocol. Integrating ROS 2 with them today means a bridge node
([ROS2_AUTOSARAP_Bridge_Converter](https://github.com/azu-lab/ROS2_AUTOSARAP_Bridge_Converter)
and similar), which becomes both a bottleneck and a single point of failure, and
which never makes the ROS graph itself SOME/IP native.

**Goal**: put all communication on SOME/IP with no change to the ROS 2
application, from `RMW_IMPLEMENTATION=rmw_someip_cpp` alone.

**Final demonstration**: a native vsomeip application with no ROS dependency and
a ROS 2 application, in separate Docker containers, communicating over SOME/IP
through docker compose.

## 2. The central problems

| # | Problem |
|---|---|
| **C1** | ROS 2 topic names are arbitrary strings; SOME/IP has only 16-bit numeric IDs |
| **C2** | A SOME/IP-SD entry carries numeric IDs, a version and a TTL, with nowhere for a type name, QoS or node name |
| **C3** | vsomeip expects a static JSON configuration; ROS 2 creates topics at runtime |
| **C4** | SOME/IP has no equivalent of durability, deadline, lifespan or history |
| **C5** | AUTOSAR SOME/IP Transformer, or CDR wrapped in a payload? |

## 3. Phase 0 measurements

C3 decided whether the whole thing was viable, so it was settled first with
minimal code that involves no ROS. That code is kept in [`spike/`](../spike/).

### 1. Runtime ID assignment (`spike/dynamic_id`)

| Subject | Must be declared in the JSON? | Result |
|---|---|---|
| `service` / `instance` | **Yes** | Without a declaration with a port under `services`, no SD offer is sent. The API returns `OFFER(...) (true)` and **not one packet reaches the wire**, confirmed with tcpdump |
| `event` / `eventgroup` | **No** | The runtime API alone is enough |
| The consumer's `services` | **No** | Endpoints are learned from SD |

`offer_service()` returning success while sending nothing is invisible without
following the logs. It was the worst trap of the three.

### 2. Dynamic event assignment within one service (`spike/multi_event`)

The JSON declares one service (`0x2000` and a port); three topics' worth of
events and eventgroups are created at runtime, and the consumer subscribes to
two of them.

```
0x8001 (/chatter) received 23
0x8002 (/sensor)  received  0   <- not subscribed, nothing leaked
0x8003 (/status)  received 23
```

**Conclusion**: with one service per process and one event plus one eventgroup
per topic, a single declaration suffices and topics can be created without limit.

### 3. Wildcard discovery (`spike/wildcard_discovery`)

One `request_service(SERVICE_ID, ANY_INSTANCE)` discovered every participant
offering that service ID under a different instance.

```
[consumer3] issuing one request_service(0xFF00, ANY_INSTANCE)
[consumer3] DISCOVERED instance=1
[consumer3] DISCOVERED instance=2
```

**Conclusion**: a whole ROS domain fits into **one service ID, with instance_id
identifying the participant**.

> An `ANY_INSTANCE` availability handler fires once with `instance=0xFFFF`.
> Implementations must ignore `instance == ANY_INSTANCE`.

### 4. Docker networking

| Symptom | Cause | Fix |
|---|---|---|
| Discovery never starts (`netlink: ... mc=0`) | The container has no `224.0.0.0/4` route, and vsomeip treats a multicast route's presence as the signal that the network is ready | `NET_ADMIN`, plus `ip route add 224.0.0.0/4 dev eth0` at startup |
| — | Multicast forwarding across the Docker bridge itself works, verified with `multicast_snooping=1` unchanged | Nothing to do |

vsomeip's `unicast` has to match the container's IP, so compose assigns static
addresses.

### 5. Bugs that surfaced in Phase 1d

Implementing services exposed three real bugs, none of which pub/sub alone
would have revealed.

**(a) The graph channel exceeded UDP's maximum length**

SOME/IP's UDP has a maximum message length, 1416 bytes by default. Each node
carries six parameter services plus `get_type_description`, so with only a few
nodes the graph outgrew it and the send failed with
`sei::segment_message: Dropping to big message`. The graph channel moved to TCP
(`RT_RELIABLE`): a dropped graph means a peer is never discovered, so it needs a
path that always arrives.

> **A related problem, since solved.** Large BEST_EFFORT topics such as
> PointCloud2 hit the same limit and were dropped with `Dropping to big
> message`.
>
> vsomeip's SOME/IP-TP configuration lists event IDs individually, but topic
> event IDs are numbered sequentially from `kFirstTopicEventId`, so **declaring
> a fixed number of them up front** in the generated JSON is enough — 256 by
> default, changed through `RMW_SOMEIP_TP_EVENT_COUNT`. With that, BEST_EFFORT
> messages far larger than one datagram get through with nothing dropped.

**(b) Entities are destroyed after shutdown**

`rclcpp::shutdown()` calls `rcl_shutdown()` and so `rmw_shutdown()`, and **then**
destroys the rosout publisher through `rcl_logging_rosout_fini()`. Destroying
the vsomeip session in `rmw_shutdown()` made the `rmw_destroy_publisher()` that
followed walk a destroyed `unique_ptr` and segfault. The session is now reached
only through a null-safe accessor.

**(c) The C and C++ introspections spell namespaces differently**

C says `"std_msgs__msg"`, C++ says `"std_msgs::msg"`. Only `"::"` was being
normalized, so an rclcpp publisher (`std_msgs/msg/String`) and an rclpy
subscription (`std_msgs__msg/String`) did not agree on the type name and
**nodes written in C and C++ could not communicate**. It only came to light when
`ros2 topic echo` produced nothing. Both `"::"` and `"__"` are normalized now.

### 6. Confirming pub/sub on the wire

`test/pubsub_smoke` was run across two containers and the packets on the bridge
inspected directly.

```
f000 8002    Message ID: service 0xF000, event 0x8002
0000 0022    Length 34
0000 0010    Request ID: client 0x0000, session 0x0010
01 00 02 00  ProtoVer 1, IfaceVer 0, MsgType 0x02 NOTIFICATION, RetCode E_OK
0001 0000    payload begins: CDR little-endian encapsulation header
1200 0000    CDR string length 18, including the NUL
"Hello SOME/IP: 14\0"
```

The service ID is `0xF000 + domain_id` as designed, and event `0x8002` is the
second sequential assignment; `0x8001` went to the rosout publisher. The CDR
payload is visible inside the SOME/IP NOTIFICATION frame.

**A constraint this revealed**: an rclcpp Node creates six parameter services
and `~/get_type_description` by default, so **no standard node starts until
services are implemented**. That is as unavoidable as publishers, so services
were brought forward into Phase 1d. `test/pubsub_smoke` disables them through
NodeOptions and a parameter override to isolate the pub/sub path.

## 4. Architecture

```
┌─────────────────────────────────────────────────────────┐
│  rclcpp / rclpy / rcl  (unchanged)                       │
├─────────────────────────────────────────────────────────┤
│  rmw_someip_cpp                                          │
│  ┌────────────┬────────────┬───────────┬──────────────┐ │
│  │ Entities   │ GraphCache │ IdMapper  │ Serializer   │ │
│  │ pub/sub/   │ the ROS    │ event and │ CDR and      │ │
│  │ srv/client │ graph      │ eventgroup│ SOME/IP      │ │
│  ├────────────┴────────────┴───────────┴──────────────┤ │
│  │ WaitSet, on a condition variable                     │ │
│  ├──────────────────────────────────────────────────────┤ │
│  │ ConfigGenerator: vsomeip.json, written at rmw_init    │ │
│  └──────────────────────────────────────────────────────┘│
├─────────────────────────────────────────────────────────┤
│  vsomeip: application and routing manager                │
├─────────────────────────────────────────────────────────┤
│  SOME/IP-SD over UDP multicast, plus a UDP/TCP data plane│
└─────────────────────────────────────────────────────────┘
```

**Process model**: one ROS process is one `rmw_context`, one
`vsomeip::application`, one SOME/IP instance. vsomeip needs one routing manager
per host. Under Docker each container is its own network namespace, so each has
its own.

## 5. Solving C1, C2 and C3: the ID model and the graph channel

### 5.1 The ID model

**One SOME/IP service for the whole ROS domain**, with participants told apart
by instance_id.

| Element | Value | Declared in the JSON? |
|---|---|---|
| Service ID | `0xF000 + (domain_id & 0xFF)`, configurable | **Yes** |
| Instance ID | participant ID, 16 bits of `hash(hostname, pid)`, excluding `0x0000` and `0xFFFF` | **Yes** |
| Ports | an `unreliable` and a `reliable` port per participant | **Yes** |
| Event `0x8000` / eventgroup `0x0001` | **reserved for the graph channel** | No |
| Event `0x8001+` / eventgroup `0x0002+` | assigned to ROS topics in turn | No |
| Method `0x0001+` | assigned to ROS services in turn | No |

The JSON only has to declare the participant's own service, instance and ports,
all of which are known by the time `rmw_init()` returns. Topics need no
configuration change however many are created.

**IDs are assigned sequentially**, not hashed: a plain incrementing counter
cannot collide. They change across a restart, which does not matter because the
graph channel is authoritative.

### 5.2 The graph channel

Every participant publishes its own slice of the ROS graph as a SOME/IP
**field** on event `0x8000` / eventgroup `0x0001` of its own service.

A subscriber discovers every participant with one
`request_service(SERVICE_ID, ANY_INSTANCE)` and subscribes to eventgroup
`0x0001` on each instance.

**Using a field is what matters here**: a field delivers its latest value on
subscribe, so a participant that starts later still receives the current graph.
That gives the equivalent of DDS TRANSIENT_LOCAL at the discovery layer.

The payload is a compact binary format (`detail/graph_message.hpp`):

```
uint32 magic "RSGI" / uint16 version / uint16 participant_instance_id
uint32 node_count      -> { namespace, name, enclave }
uint32 endpoint_count  -> {
    uint8  kind (pub/sub/service/client)
    string node_namespace, node_name
    string topic_name          # fully qualified ROS name
    string type_name           # "std_msgs/msg/String"
    uint16 event_id            # the value actually assigned; authoritative
    uint16 eventgroup_id
    uint16 method_id           # services and clients only
    uint8  reliability / durability / history
    uint32 depth
    uint8[16] gid
}
```

The original plan was a rosidl message package (`rmw_someip_msgs`), following
`rmw_dds_common`. But this channel is internal to the RMW and never reaches a
non-ROS SOME/IP peer, which goes through extern mode's static mapping instead,
so a whole extra package was not worth it. The leading magic number and version
make a format change detectable.

That solves C1 and C2: SOME/IP-SD carries only that a participant exists and how
to reach it, and everything ROS-specific travels on the graph channel.

### 5.3 Generating the vsomeip configuration

`rmw_init()` writes a vsomeip JSON from the ROS environment (`ROS_DOMAIN_ID` and
friends), points `VSOMEIP_CONFIGURATION` at it, and then calls
`create_application()`. It contains only what §5.1 marks as needing a
declaration, plus the SD parameters and routing settings.
`RMW_SOMEIP_CONFIG_FILE` is the escape hatch for a hand-written configuration.

**Separating domains**: the SD multicast port is offset by
`30490 + domain_id * 2`, and the service ID carries the domain too.

## 6. Native and extern mode

The demo requires talking to a native vsomeip application, so topics come in two
kinds.

| | **Native** | **Extern** |
|---|---|---|
| For | ROS to ROS | ROS to a non-ROS SOME/IP peer, such as an AUTOSAR ECU |
| IDs | assigned sequentially; the graph channel is authoritative | pinned in `someip_mapping.yaml` |
| Serialization | CDR | AUTOSAR SOME/IP Transformer |
| Peer joins the graph channel | required | not required; the RMW synthesizes the graph entry |

Extern mode is effectively a bridge built into the RMW, which removes the need
for a bridge node in its own process.

```yaml
# someip_mapping.yaml
externs:
  /vehicle/gnss:
    direction: subscribe          # ROS subscribes, the peer offers
    service_id: 0x1234
    instance_id: 0x0001
    event_id: 0x8001
    eventgroup_id: 0x0001
    type: rmw_someip_demo_msgs/msg/VehicleStatus
    serialization: someip_native
```

Named by `RMW_SOMEIP_MAPPING_FILE`. An extern topic's service ID differs from
the participant's, so the generated JSON gains an entry for it as well — with a
port, when the ROS side offers.

## 7. Solving C4: the QoS mapping

| ROS 2 QoS | Mapping | Implemented in |
|---|---|---|
| `RELIABLE` | TCP endpoint, the `reliable` port | vsomeip configuration |
| `BEST_EFFORT` | UDP endpoint, the `unreliable` port | vsomeip configuration |
| `TRANSIENT_LOCAL`, depth 1 | **offered as `ET_FIELD`**, which delivers the latest value on subscribe | vsomeip, natively |
| `TRANSIENT_LOCAL`, depth above 1 | `ET_EVENT`, a send history, and a `notify_one()` replay from `register_subscription_handler` (§21) | the RMW |
| `KEEP_LAST(depth)` | receive queue bounded to depth | the RMW |
| `KEEP_ALL` | unbounded queue | the RMW |
| `DEADLINE` | send and receive intervals watched on the steady clock | the RMW, in a watchdog |
| `LIFESPAN` | expired messages dropped from the receive queue | the RMW |
| `LIVELINESS: AUTOMATIC` | the graph channel and SD availability count as alive | SD and the RMW |
| `LIVELINESS: MANUAL_BY_TOPIC` | the publisher watches its lease; the subscriber judges from arrival gaps (§21) | the RMW |

Unlike DDS, SOME/IP does not refuse to connect on mismatched QoS. **The RMW
checks compatibility itself against the graph cache** and does not `subscribe()`
to an incompatible peer. Without that, a BEST_EFFORT publisher and a RELIABLE
subscriber would silently connect.

### 7.1 Deriving the QoS events

SOME/IP has nothing like the DDS StatusCondition, so every event
`rmw_event_type_is_supported()` reports is maintained by the RMW itself.

| Event | Derived from |
|---|---|
| `PUBLICATION_MATCHED` / `SUBSCRIPTION_MATCHED` | graph channel changes, counting only QoS-compatible peers |
| `LIVELINESS_CHANGED` | under AUTOMATIC, a matched publisher is a live publisher; under MANUAL_BY_TOPIC, the subscriber times arrivals (§21) |
| `LIVELINESS_LOST` | always 0 under AUTOMATIC, since a running process cannot lose it; under MANUAL_BY_TOPIC, the watchdog sees the lease expire |
| `OFFERED_DEADLINE_MISSED` / `REQUESTED_DEADLINE_MISSED` | the QoS watchdog, ticking every 20 ms |
| `OFFERED_/REQUESTED_QOS_INCOMPATIBLE` | comparing QoS on the graph channel (§21) |
| `MESSAGE_LOST` | **unsupported**: detecting it needs a sender-side sequence number, which means changing the wire format (§21) |

**A deadline** detects that nothing happened for a while, which no
message-driven path can notice; it takes a thread watching the clock (the QoS
watchdog in `rmw_context_impl_s`). It starts only once an entity with a deadline
exists, so the common case adds no thread at all.

The clock starts at **the first write or receipt**. Counting from entity
creation would report a missed deadline whenever discovery took longer than the
deadline itself, even though it was never missed. DDS also counts a per-instance
deadline from the first sample, so this matches.

**Liveliness** can be read off the graph channel only because AUTOMATIC means
"the participant's process is alive": a dead process shows as SD
availability=false, and a destroyed entity as an updated peer graph.
MANUAL_BY_TOPIC turns on time and is handled separately (§21).

**Lifespan** is measured in DDS from the sample's source timestamp, but these
payloads are plain CDR — AUTOSAR in extern mode — with nowhere to put a send
time. Adding a header would break both CDR compatibility with Fast DDS and byte
compatibility with a non-ROS peer. Elapsed-since-receipt is used instead, and
expired messages are dropped in `rmw_take`. Messages live slightly longer than
they should by the network delay, but a backlog on a slow subscriber expires
correctly, which is the case that matters.

## 8. Solving C5: serialization

The serializer is abstracted with two implementations.

**(a) CDR** walks the `rosidl_typesupport_introspection_c/cpp` `MessageMembers`
at runtime, the same approach as `rmw_cyclonedds_cpp`. No per-type code
generation, so every ROS message works. `rmw_get_serialization_format()` returns
`"cdr"`, which keeps rosbag2 interoperable. Used in native mode.

**(b) SOME/IP native** follows the AUTOSAR SOME/IP Transformer, and is used in
extern mode. The traversal is shared (`detail/introspection_traits.hpp`) but the
encoding rules differ enough that the implementations are separate
(`detail/someip_serializer.cpp`):

| | CDR | SOME/IP |
|---|---|---|
| Byte order | little endian | **big endian** |
| Alignment | to the type width, padded | **packed, no padding** |
| Encapsulation header | 4 bytes | **none** |
| Dynamic array length | **element count** (uint32) | **byte count** (uint32) |

The last row is what complicates reading: with no element count, anything of
variable size (strings, nested structs) has to be skipped over and counted
first.

## 9. Mapping the RMW primitives

### Publishers and subscriptions

| RMW | SOME/IP |
|---|---|
| `rmw_create_publisher` | assign an event and eventgroup, `offer_event`, update the graph channel |
| `rmw_publish` | serialize, `create_payload`, `notify()` |
| `rmw_create_subscription` | look the topic up on the graph channel, then `request_event` and `subscribe` on the matching instance |
| `rmw_take` | pop from the receive queue, which the message handler pushes to |
| `rmw_count_publishers` and friends | queries against GraphCache |

### Services and clients

DDS implementations build a service out of two topics, one per direction, but
**SOME/IP has request/response natively**, so the mapping is direct.

| RMW | SOME/IP |
|---|---|
| `rmw_create_service` | register a handler for (service, instance, method_id) |
| `rmw_send_request` | `create_request()` then `send()`, correlating the SOME/IP request ID with `rmw_request_id_t` |
| `rmw_take_request` | pop from the handler's queue, keeping the request ID |
| `rmw_send_response` | `create_response(original)` then `send()`; vsomeip carries the request ID over |
| `rmw_take_response` | pop from the client's queue and correlate by request ID |

Actions need nothing extra: `rcl_action` builds them on services and topics.

### Wait sets

vsomeip invokes its callbacks on its own threads, so `rmw_wait` is built on a
condition variable. Each entity owns a queue and a mutex, and a callback only
**pushes and notifies** — calling user code from inside one would deadlock.

## 10. Layout

```
rmw_someip/
├── docker/                    # base.Dockerfile, test.Dockerfile
├── spike/                     # the Phase 0 measurements (§3)
├── rmw_someip_cpp/
│   └── src/
│       ├── rmw_init.cpp       # context and vsomeip application lifecycle
│       ├── rmw_node.cpp
│       ├── rmw_publish.cpp, rmw_subscribe.cpp
│       ├── rmw_service.cpp, rmw_client.cpp
│       ├── rmw_wait.cpp, rmw_event.cpp
│       ├── rmw_graph.cpp      # graph introspection
│       ├── rmw_serialize.cpp, rmw_misc.cpp
│       └── detail/
│           ├── someip_endpoint.hpp   # the seam between the RMW and vsomeip
│           ├── someip_session.cpp    # its production implementation
│           ├── context_impl.cpp      # the graph channel, routing, QoS events
│           ├── graph_cache.cpp, graph_message.cpp
│           ├── participant.cpp       # IDs and port reservation
│           ├── config_generator.cpp  # writes the vsomeip JSON
│           ├── type_support.cpp      # the CDR serializer
│           ├── someip_serializer.cpp # the AUTOSAR serializer
│           ├── extern_mapping.cpp    # someip_mapping.yaml
│           ├── entity_data.hpp, event_data.hpp, message_pool.hpp
│           └── cdr.hpp, wait_set.hpp, qos_compat.hpp, rmw_checks.hpp
├── demo/                      # the extern mode demo (§12)
├── test/                      # see test/README.md
├── tools/ci/                  # what CI and a developer both run
└── docs/design.md
```

## 11. Implementation phases

Extern mode was prioritized because the demo depends on it.

| Phase | Contents | Outcome |
|---|---|---|
| **0. Groundwork** | The measurements in §3, and the Docker base image | Done |
| **0b. Skeleton** | Every rmw symbol stubbed, dlopenable from `rmw_implementation` | Errors propagate from rmw through rcl to rclcpp with line numbers |
| **1a. Context and node** | `rmw_init` creating the vsomeip application, nodes, guard conditions, wait sets | `rclpy.init()` succeeds and vsomeip comes up as far as the SD endpoint |
| **1b. Serializer** | Generic CDR over the rosidl introspection, C and C++ | Interoperable with `rmw_fastrtps_cpp` in every case of `test/cdr_compat_test` |
| **1c. Pub/sub, native** | ID assignment, graph channel, GraphCache, publishers, subscriptions | Pub/sub across containers, with SOME/IP NOTIFICATION plus CDR confirmed on the wire (§3) |
| **1d. Services and clients** | Mapped onto SOME/IP methods, request IDs correlated | Unmodified `demo_nodes_cpp` works, as does `ros2` CLI graph introspection |
| **2. Extern mode** | `someip_mapping.yaml`, the SOME/IP native serializer | A native vsomeip application and a ROS 2 application communicate both ways across containers ([demo/](../demo/)) |
| **3. Graph introspection** | `rmw_get_node_names`, `rmw_get_topic_names_and_types`, `rmw_count_*`, `*_by_node`, type hashes | `ros2 node list`, `topic list`, `node info` and `topic info -v` all correct (§20) |
| **4. QoS** | TCP/UDP by reliability, TRANSIENT_LOCAL, depth, BEST_AVAILABLE resolution, matched / deadline / liveliness events, compatibility checking | `test_quality_of_service` passes, with `test_deadline` and `test_automatic_liveliness_changed` actually running rather than skipped (§17) |
| **5. Conformance** | The official suites in CI | **All four suites pass**: `test_rmw_implementation`, `test_communication`, `test_rclcpp` and `test_quality_of_service` (§15 to §17). `test_communication`'s rclcpp/rclpy pairs still fail intermittently, on a race inside rclpy (§19) |

## 12. The demo

```
┌──────────────────────────┐        ┌──────────────────────────┐
│ container: someip_app     │        │ container: ros2_app       │
│                           │        │                           │
│  native vsomeip app       │        │  ROS 2 node               │
│  no ROS dependency        │        │  RMW_IMPLEMENTATION=      │
│                           │        │    rmw_someip_cpp         │
│  offer_service(0x1234)    │        │                           │
│  notify(event 0x8001)     │        │  rclcpp::Subscription     │
│                           │        │    on "/vehicle/status"   │
│  172.30.0.11              │        │  172.30.0.12              │
└───────────┬──────────────┘        └──────────┬───────────────┘
            │                                   │
            └────────── someip_net (bridge) ────┘
                   SOME/IP-SD 224.244.224.245:30490
                   plus a UDP data plane
```

An extern entry in `someip_mapping.yaml` pins `/vehicle/status` to service
`0x1234` / event `0x8001`, and the ROS side receives it with SOME/IP native
serialization. **The ROS 2 source never mentions SOME/IP**, which is the
difference from a bridge.

Both containers need `NET_ADMIN` and a multicast route (§3).

## 13. How this is verified

**End to end**
```bash
docker compose -f demo/compose.yaml up
```

**On the wire.** A ROS log cannot show whether the traffic really goes over
SOME/IP, so this is not optional.
```bash
BR="br-$(docker network inspect <net> -f '{{.Id}}' | cut -c1-12)"
docker run --rm --network host --privileged rmw_someip/base:jazzy \
  tcpdump -i "$BR" -nn 'udp port 30490 or portrange 30500-30600'
```
Wireshark's SOME/IP dissector shows the SD offers and subscribes and the data.

**Conformance**: `test_rmw_implementation` for the rmw contract,
`test_communication` for cross-language communication,
`test_quality_of_service`, and the `rcl` and `rclcpp` suites.

**Benchmark**: against `rmw_fastrtps_cpp` (§24).

## 14. Known risks

| Risk | Impact | Mitigation |
|---|---|---|
| ~~Dynamic ID assignment in vsomeip~~ | — | **Resolved** (§3). Events and eventgroups are fully dynamic |
| ~~Port collisions~~ | — | **Resolved** (§20). A lock file under `/tmp` reserves a port for the life of the process, so simultaneous starts no longer collide |
| `instance_id` collision across hosts | The peer is never discovered | A collision on one host is very unlikely. Across hosts vsomeip catches it during discovery and logs `Rejecting routing info ... offered previously on this node` (§20). Split by domain or by host to avoid it |
| The routing manager is a single point of failure | All communication on the host stops | Detected and reported explicitly |
| Discovery overhead | SD traffic grows with the number of topics | One service per participant, so the number of offers scales with processes rather than topics. Already mitigated by the design |
| The 16-bit event space | Bounds the number of topics per process | Not a practical limit |

## 15. Results: test_rmw_implementation

Running the official
[`test_rmw_implementation`](https://github.com/ros2/rmw_implementation) against
`rmw_someip_cpp`. How to fetch and run it is in
[`test/README.md`](../test/README.md).

At first several test binaries crashed or hung and others failed. Every binary
now runs to completion with no failure and nothing skipped; the last skips went
away once loaned messages were implemented (§22).

### Bugs the suite found

None of these showed up in the demos.

**(a) Handle implementation identifiers were not checked**

`rmw_destroy_publisher()` and others did not check the *node's*
`implementation_identifier`. A node from another implementation was accepted,
the caller retried, and the double free segfaulted. Argument checking is part of
the rmw contract, not a nicety: **every function taking a handle must verify its
identifier**.

**(b) The bulk-copy optimization was applied to dynamic sequences**

The "copy a contiguous region with one memcpy" path meant for fixed-size arrays
was also being taken for dynamic sequences, where the pointer refers to the
container: reading its raw bytes emitted the container's internal pointers. The
receiver read those as an element count and tried a huge resize, corrupting the
heap.

**`std::vector<bool>` is also bit-packed** and cannot hand out element pointers.
rosidl leaves `get_const_function` null for such types and provides
`fetch_function` and `assign_function`, which copy the value; those were not
being used.

**(c) The event array is passed differently from the others**

In `rmw_wait()`, subscriptions, services and clients arrive as each handle's
`data`, the implementation's own type, but **an event arrives as the
`rmw_event_t` handle itself**. Casting it directly locked a mutex at a garbage
pointer and hung the whole executor.

**(d) Entities are still destroyed after shutdown**

`rmw_context_fini()` must reject a context that was not shut down, and node
creation must fail afterwards. Whether a session exists cannot stand in for
either.

### The CDR compatibility check was changed

Comparing bytes directly against `rmw_fastrtps_cpp` turned out to be too strict.
Fast DDS leaves uninitialized bytes in struct padding, whose contents CDR leaves
unspecified, and puts an over-estimated size in `buffer_length` rather than what
it actually wrote.

The check became a **round trip across implementations**: can Fast DDS read what
this one wrote, and the reverse. That asks about interoperability directly and
is not disturbed by padding or size estimates. Every case passes, including
`test_msgs`'s bounded and unbounded sequences, nested types and fixed arrays.

### What the last failures taught

**(e) A matched event is a cumulative status, not a queue**

`rmw_take_event()` does not pop one queued event. Like a DDS StatusCondition it
returns the running total plus the delta since the last read and clears the
delta, so **`taken` is true even when nothing changed**. Implemented as a queue,
it returned `taken=false` when nothing had fired.

**(f) A matched count counts only QoS-compatible peers**

SOME/IP does not refuse to connect on mismatched QoS, so what
`rmw_publisher_count_matched_subscriptions()` should report can only come from
the RMW deciding compatibility itself.

**(g) The compatibility check has to warn about a SYSTEM_DEFAULT liveliness too**

`rmw_qos_profile_check_compatible()` was only warning about SYSTEM_DEFAULT
reliability and durability. That made two default profiles, whose liveliness is
SYSTEM_DEFAULT, compare as OK, disagreeing with implementations that return
WARNING. Asking `rmw_fastrtps_cpp` the same question is what revealed it.

**(h) With avoid_ros_namespace_conventions, names must not be validated**

The flag means the name is a native one that ignores ROS conventions, so a
relative name like `"test"` has to be accepted.

**(i) `rmw_service_server_is_available()` returns ERROR for a null argument**

Other functions return INVALID_ARGUMENT; this one and `rmw_destroy_wait_set()`
return `RMW_RET_ERROR`. The contract has to be checked per function.

## 16. Results: test_communication

Every test passes. Only same-RMW pairs are run; a fastrtps/someip pair cannot
communicate by construction and is excluded.

> The rclcpp/rclpy pairs used to fail often, because `rmw_wait()` could return
> TIMEOUT from an infinite wait (§19). What remains after the fix is a rare
> shutdown race inside rclpy.

**Pub/sub works in C++ for every message type**: Arrays, BasicTypes,
BoundedPlainSequences, BoundedSequences, Builtins, Constants, Defaults, Empty,
MultiNested, Nested, Strings, UnboundedSequences and WStrings. Services
(requester_replier), actions (action_client_server), serialized publishing and
nested types all pass too.

### Bugs this suite found

**(j) Two processes on one host could not communicate**

The most significant find. Each process wrote `"routing": "<its own app name>"`
into its generated configuration, so the second one failed `init()` with

```
Application: rmw_someip_55 configured as routing but other routing manager
present. Won't instantiate routing
```

vsomeip allows exactly one routing manager per host.

**Omitting** `"routing"` makes vsomeip treat the first application to start as
the routing manager and the rest as proxies, so it is left out by default. A
setup with a dedicated routing daemon names it in `RMW_SOMEIP_ROUTING_HOST`.

> The demos had always used separate containers, and therefore separate `/tmp`
> directories and separate routing managers, which hid this completely. It only
> appeared once `test_communication` started a publisher and a subscriber as two
> processes.

**(k) wstrings were unimplemented**

A CDR wstring is a uint32 character count followed by the characters, **each
4 bytes**. ROS represents wstrings as UTF-16 (`u16string`), but
`rosidl_typesupport_fastrtps_cpp` converts to `std::wstring` before encoding and
`wchar_t` is 4 bytes on Linux. Verified against its output:
`u"ab"` becomes `02 00 00 00 | 61 00 00 00 | 62 00 00 00`.

**(l) Destroying the vsomeip session in rmw_shutdown was too early**

DDS implementations keep the participant until `rmw_context_fini()`. rcl keeps
destroying entities after shutdown, and the application loop may still call
publish, for instance while rclpy handles a SIGINT. Destroying the session there
failed those with "vsomeip application is not available" and made the caller
throw. Shutdown now only stops the SD offer; destruction waits for fini.

### What test_rclcpp and test_quality_of_service added

**(m) vsomeip had been allowed to take the signal handlers**

The base image built vsomeip with `ENABLE_SIGNAL_HANDLING=1`, so vsomeip
installed its own SIGINT and SIGTERM handlers and **the application's stopped
working**. An RMW is a library and must not intercept an application's signal
handling. `test_rclcpp`'s `test_sigint_handler` and `test_sigterm_handler`
detect it: the expected `Custom sigint handler called.` never appears. Turning
the option off made `test_rclcpp` pass in full and removed several of
`test_communication`'s failures.

**(n) Resolving BEST_AVAILABLE QoS is the RMW's job**

Resolving `rclcpp::BestAvailableQoS` to concrete values is the **RMW
implementation's** responsibility, not rcl's, and `rmw_dds_common` provides the
shared utility (`qos_profile_get_best_available_for_topic_{publisher,subscription}`).
Not calling it left `*_get_actual_qos()` and `*_info_by_topic()` returning
BEST_AVAILABLE.

**(o) The graph channel carried only part of the QoS**

`rmw_get_publishers_info_by_topic()` is contractually required to return the
actual QoS, so deadline, lifespan, liveliness and lease duration have to travel
alongside the reliability and durability that compatibility depends on.
Resolving BEST_AVAILABLE reads them too.

**(p) One request can be answered more than once**

rmw does not require exactly one response per request. rclcpp's deferred-response
pattern calls `send_response` repeatedly for the same `request_header`:
`test_rclcpp`'s `add_two_ints_defered_cb_with_handle` calls it from a
zero-period timer. Discarding the pending entry after answering made the second
call fail. DDS implementations simply send the duplicate response, so the entry
is kept, with a cap that drops the oldest.

### What the last failures were

**(q) A null pointer was passed when assigning an empty string**

`rosidl_runtime_c__{String,U16String}__assignn()` is specified to return false
for a null value pointer, and an empty `std::vector`'s `data()` may be null, so
assigning an empty wstring failed — `failed to assign a wstring field` on the
rclpy side. `std::u16string::assign` accepts it, which is why the C++ path never
showed the problem.

**(r) rmw_shutdown() has to return quickly**

rclpy's `Context.try_shutdown()` is `if ok(): shutdown()`, which races the
`rcl_shutdown` in the signal handler. Calling `stop_offer_service()` inside
`rmw_shutdown()` widened that window until it threw "rcl_shutdown already
called". All teardown moved to `rmw_context_fini()`.

**(s) Use after free while destroying the context**

`~rmw_context_impl_s` destroys members in reverse declaration order, so
`graph_cache` went before `session`, the vsomeip application. With a dispatch
thread still running a callback, that reaches a destroyed object. **SIGINT to 10
processes at once crashed 7 of them in `free()` inside
`GraphCache::update_participant()`.** The destructor now calls `session.reset()`
explicitly, first.

> More processes mean more graph traffic and a better chance of reproducing it.
> Two processes never did; `test_rclcpp`'s `test_n_nodes`, with 11, reproduced
> it reliably.

## 17. Implementing the deadline and liveliness events

`test_quality_of_service`'s `test_deadline` and
`test_automatic_liveliness_changed` both begin by checking
`rmw_event_type_is_supported()` and `GTEST_SKIP()` if the event is unsupported.
Passing and having verified anything are therefore different things, and getting
them to run at all meant implementing the events first.

The approach is in §7.1. What follows only became apparent once it was built.

**(t) `rmw_take_event()`'s `event_info` is a different struct per kind**

Only `rmw_matched_status_t` was being written. The kind has to be dispatched on
`rmw_event_t::event_type` and `rmw_liveliness_changed_status_t`,
`rmw_{offered,requested}_deadline_missed_status_t` and
`rmw_liveliness_lost_status_t` written accordingly. The caller only allocated
room for the kind it asked about, so getting it wrong writes past the end.

**(u) `total_count_change` must always be 1**

`test_deadline` asserts `ASSERT_EQ(1, event.total_count_change)` inside the
callback. Incrementing once per elapsed period fails the moment a busy executor
lets two periods accumulate. One watchdog tick now fires at most once, and more
than one period behind it restarts from the current time rather than catching up.

**(v) A deadline starts at the first write or receipt, not at entity creation**

Counting from creation reports one extra miss whenever discovery took longer
than the deadline. `test_deadline` checks the event count exactly — `total_count`
must be 4 — so that extra one fails it.

**(w) A publisher's deadline has nothing to do with the entity being alive**

`QosTestPublisher::setup_stop()` stops the timer and leaves the publisher
entity in place, so OFFERED_DEADLINE_MISSED has to fire while publishing is
stopped. Watching the publish path cannot detect that, which is why a thread
watching the clock is needed.

### Results

| Test | Before | After |
|---|---|---|
| `test_deadline` | skipped | **passes**, with exactly 4 deadline events on each side |
| `test_automatic_liveliness_changed` | skipped | **passes**, with exactly 2 events, alive 0→1→0 |
| `test_lifespan` | passes | passes |
| `test_best_available` | passes | passes |

## 18. Getting the linters to pass

`rmw_someip_cpp` declared `ament_lint_common` but copyright, cpplint and
uncrustify had been failing all along, unnoticed because nothing built with
`BUILD_TESTING=ON`. 51 cpplint complaints and 7 files' worth of uncrustify
differences later, all six pass. How to run them is in
[`test/README.md`](../test/README.md).

Most of it was mechanical; two points needed a decision.

**A ternary puts `?` and `:` at the end of the line**

ROS 2's uncrustify configuration wants them at the end rather than the start.
`ament_uncrustify --reformat` does the whole lot.

**A chain of `else if` is incompatible with cpplint**

A `} else if (...)` whose condition spans several lines puts the opening brace on
the next line, which cpplint's `readability/braces` reads as an `else` with a
brace on only one side. The chain in `rmw_qos_profile_check_compatible()` was
split into two early-return functions instead, leaving the order and the results
unchanged.

### A side effect: test_communication's flakiness became visible

To confirm the lint changes had not affected communication, both versions were
built in separate worktrees and the affected test run repeatedly on each: **it
failed at a similar rate before and after**. So it was not a regression; it had
always been flaky (see the note in §16). A single run cannot tell the difference.

## 19. Why the rclcpp/rclpy pairs were flaky

Tracking down the flakiness from §18 led not to a discovery race but to **a bug
in `rmw_wait()`**. The failure looked like this:

```
[test_requester-3] [ERROR] [rclcpp]: caught std::exception exception in
                   GraphListener thread: rcl_wait unexpectedly timed out
[test_requester-3] terminate called ... what(): rcl_wait unexpectedly timed out
[ERROR] [test_requester-3]: process has died [pid 806, exit code -6]
```

rclcpp's `GraphListener` waits forever with `rcl_wait(&wait_set_, -1)`, and rcl
turns a negative timeout into `rmw_wait(..., wait_timeout = nullptr)`. So
**`RMW_RET_TIMEOUT` must never be returned when `wait_timeout` is null**.
Returning it makes rclcpp throw `std::runtime_error`, which leaves the
GraphListener thread and takes the process down through `std::terminate` and
SIGABRT.

### Why TIMEOUT was being returned

`WaitSetImpl::triggered_` is a level flag saying somebody woke the set; **it does
not record who**. So the flag can outlive the reason for it:

1. `wait()` returns and clears `triggered_`.
2. **Immediately after**, before `detach()`, a message arrives: it is queued and
   `triggered_` is set again.
3. The readiness scan picks that message up. Correct so far.
4. The caller drains it with `rmw_take()`.
5. The next `rmw_wait()` returns at once on the leftover `triggered_`, with
   nothing to report, and answers `RMW_RET_TIMEOUT`.

The window is easiest to hit just after startup, when the graph is changing
rapidly and `publish_local_graph()` runs repeatedly.

### The fix

Narrowing the window does not help, since a trigger can also land right after
the scan. **Waiting again on the remaining time** is the only fix; an infinite
wait retries until something is ready. Detaching the entities and nulling the
slots that are not ready happens once, after the loop.

### Result

On the two tests that were failing, `test_requester_replier__rclcpp__rclpy` and
`test_action_client_server__rclcpp__rclpy`, repeated runs fail far less often
than before, though not never.

The failures that remain are something else — a shutdown race inside rclpy:

```
rclpy.try_shutdown() → context.shutdown()
RCLError: failed to shutdown: rcl_shutdown already called on the given context
```

`Context.try_shutdown()` is `if ok(): shutdown()`, and if the signal handler's
`rcl_shutdown` completes between those two calls it throws. That is rcl's
internal state and no concern of the RMW; `rmw_shutdown()` here already does
nothing but set a flag, so there is no window left to narrow from this side.

> The same tests under `rmw_fastrtps_cpp` did not fail in a comparable number
> of runs. That is too few to be significant, but neither has it been shown
> that Fast DDS can hit the same race. This is left open.

## 20. Filling in graph introspection, and ID collisions

Four gaps that could be closed with information already available: either the
graph channel was carrying it and nothing used it, or the design described it
and nothing implemented it.

### (a) `ros2 node info` was always empty

All four of `rmw_get_{publisher,subscriber,service,client}_names_and_types_by_node`
validated their arguments and returned an empty list. The graph channel carries
each endpoint's node, so filtering in `GraphCache::names_and_types_of_node()`
was all it took.

`no_demangle` is a DDS-specific notion — whether to strip ROS naming conventions
— and makes no difference here, where ROS names are carried verbatim.

### (b) The type hash was not carried

`ros2 topic info -v` always showed an INVALID "Topic type hash". Graph channel
v3 adds the REP-2011 (RIHS) hash.

The hash lives on the **outer** typesupport handle rcl passes, the dispatcher.
The inner introspection handle usually has it too, but not always, so outer is
tried before inner. Typesupports without one exist — dynamic types — so failing
to find it is not an error.

```
Topic type hash: RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
```

### (c) Port collisions, which did real damage

Ports were derived directly from the instance ID, modulo a small span. **With so
few slots**, a collision becomes likely as soon as several processes share a
network namespace. A collision makes one vsomeip fail to bind and it cannot
communicate at all. `test_rclcpp`'s `test_n_nodes` starts enough processes to
hit it.

The first attempt was to probe with bind and take the port if it was free, but
**starting many processes at once still produced a collision**: the probe closes
the socket, and another process can see the port as free before vsomeip binds it.

It now holds a `flock` on a file under `/tmp` for the life of the process.
vsomeip already uses `/tmp` for local routing, so what `/tmp` is shared with
matches what a port can collide with — the network namespace — and this excludes
other rmw_someip processes on the host reliably. The bind probe stays to catch
ports held by something that is not rmw_someip.

Starting many processes at once now gives every one of them distinct ports.

### (d) vsomeip catches an instance_id collision first

The graph channel gained a 64-bit participant fingerprint (guid), so "same
instance_id, different guid" is reported as a collision.

**In practice nothing reaches that code.** Forcing a collision with matching
hostname and pid in two containers made both log

```
rmi::add_routing_info: Rejecting routing info. Remote: 172.45.0.3 is trying to
offer [f000.0770.0.0] on port 32904 offered previously on this node
```

during discovery, and no graph message arrived at all. So a collision shows up
in vsomeip's log. The guid stays anyway, so that one which does get through is
not mistaken for this participant's own echo and dropped in silence.

### (e) Subscriptions were never cancelled

`remove_subscription()` never called `unsubscribe()`. Several subscriptions can
share a topic, so cancelling unconditionally is wrong, and without a refcount
the choice made was to never cancel — which piles up vsomeip subscriptions in an
application that creates and destroys them.

They are now refcounted per (service_id, instance_id, event_id, eventgroup_id),
and only the transition to zero calls `unsubscribe()` and `release_event()`.
Extern-mode subscriptions went into the same table; previously they were not
even recorded.

Verified: with two subscriptions on one topic, destroying one leaves the other
receiving; after 30 create/destroy cycles a new subscription still receives.

## 21. Completing the QoS

### (a) TRANSIENT_LOCAL above depth 1

**A vsomeip field holds one value.** It has no notion of history, so it cannot
express a depth above 1. Those are offered as `ET_EVENT` instead, with the RMW
keeping a send history, detecting new subscribers through
`register_subscription_handler`, and replaying to that subscriber alone with
`notify_one()`, oldest first.

Sender and subscriber have to agree on the event type, so the decision lives in
one function, `event_type_for(durability, depth)`. The subscribing side reads
**the publisher's** durability and depth from the graph channel, not its own.

The replay does not happen inside the subscription handler. A request is queued
for the QoS watchdog instead, because the handler is called from a vsomeip
thread holding vsomeip's own lock, and sending from there is best avoided.

**Neither fields nor replay reach the same process.** Delivery within a process
goes through `deliver_locally()`, which never touches SOME/IP and has no notion
of history. Left alone, the history would arrive from another process but not
from the same one, so `deliver_local_history()` delivers it when the
subscription is created. That is why the history is kept even at depth 1.

### (b) MANUAL_BY_TOPIC liveliness

The semantics are that writing to the topic asserts liveliness. So:

- **The publisher** extends its lease on `rmw_publish` and
  `rmw_publisher_assert_liveliness()`, and raises `LIVELINESS_LOST` when it
  expires.
- **The subscriber** decides from whether anything arrived from that publisher
  within the lease, which adds nothing to the wire.

`rmw_publisher_assert_liveliness()` stopped returning `RMW_RET_UNSUPPORTED` and
was implemented. Updating a local counter alone **would not make the publisher
look alive to subscribers**, though — they decide from arrivals, so asserting
without sending anything would still be reported not alive. So the assertion
**is a zero-length notification**. CDR always begins with a 4-byte encapsulation
header, so no ordinary native-mode message is empty and nothing collides.

> In extern mode zero length is a legitimate payload: the AUTOSAR encoding of a
> message with no fields is zero bytes. A non-ROS peer knows nothing about
> liveliness assertions either, so extern subscriptions skip this reading.

**`LIVELINESS_LOST` fires once, when it is lost.** A deadline counts one miss
per period, but liveliness must not keep incrementing while it stays lost. The
first implementation fired again on every lease period for as long as the
silence lasted. `DeadlineTracker::set_one_shot()` suppresses it until the next
write.

**The alive count has to be computed in one place.** AUTOMATIC follows graph
changes and MANUAL_BY_TOPIC follows time, and writing both into
`LIVELINESS_CHANGED` separately makes them fight: the graph sets alive back to 1
while the watchdog sets it to 0. That really did emit a spurious event with zero
deltas. `check_subscription_liveliness()` computes both together.

### (c) Incompatible-QoS events

The graph channel carries the peer's complete QoS, so the same check used for
match counts also counts the peers rejected as incompatible. The offending
policy (`last_policy_kind`) has to be reported too, so the check was extracted
into [`detail/qos_compat.hpp`](../rmw_someip_cpp/src/detail/qos_compat.hpp).

`GraphCache` used to call the public `rmw_qos_profile_check_compatible()`, which
collapses WARNING and ERROR into one enum and never names the policy. With the
check extracted, the public API, the match counts and the incompatibility events
always reach the same conclusion.

### (d) MESSAGE_LOST stays unsupported

Detecting it needs a sequence number from the sender. The session ID in the
SOME/IP header is per (service, instance) and increments across **all** of that
instance's events, so it says nothing about one topic. A sequence number of our
own in the payload would work, but that changes the CDR bytes and breaks both
interoperability with `rmw_fastrtps_cpp` and byte compatibility in extern mode.
Not worth it.

### Verification

The official suites contain no test for any of this, hence
[`test/qos_events/`](../test/qos_events/).

| Check | Result |
|---|---|
| TRANSIENT_LOCAL depth 5, late subscriber in another process | 5 history entries, oldest first |
| The same at depth 1 and 5, late subscriber in the same process | 1 and 5 received |
| MANUAL_BY_TOPIC, 1 s lease | the subscriber's `LIVELINESS_CHANGED` goes (alive 1, not_alive 0) → (0, 1) → (0, 0) |
| The same, publisher side | `LIVELINESS_LOST` fires **once** |
| BEST_EFFORT publisher against a RELIABLE subscription | both raise `*_QOS_INCOMPATIBLE` with `last_policy_kind = RELIABILITY` |

## 22. Loaned messages

The six `rmw_borrow_loaned_message()` functions are implemented, which takes
`test_rmw_implementation` to **nothing skipped**; the loan tests had been
skipping on `can_loan_messages = false`.

### This is not shared-memory transport

The limitation first. **vsomeip's local path is a UNIX domain socket and has no
shared memory.** CDR still goes over the wire, so this is not zero copy in the
sense of the receiver reading the memory the sender wrote.

The rmw contract is that the caller borrows memory from the middleware and says
nothing about transport, so the contract is met. What is actually saved is **the
application's own allocation and destruction of the message**, and no more.

Real zero copy would mean a separate shared-memory path for same-host delivery,
bypassing SOME/IP for local traffic — which contradicts the premise of putting
all communication on SOME/IP, so it is not taken.

### Implementation

Even without per-type code generation, the introspection's `size_of_`,
`init_function` and `fini_function` are enough to instantiate any type. The pool
built on them is
[`detail/message_pool.hpp`](../rmw_someip_cpp/src/detail/message_pool.hpp).

A returned message is `fini`ed and `init`ed again. `fini` frees the inner
buffers of strings and sequences, so **reuse only saves the outer allocation** —
but reusing without clearing would leak the previous value into the next loan,
so that cost cannot be avoided.

Outstanding loans are tracked in a set. Returning a pointer that was never lent
has to produce `RMW_RET_ERROR`, which the official suite checks, and letting it
through would try to free a variable on the application's stack.

The destructor destroys outstanding loans too, so a caller that forgot to return
one does not leak when the publisher or subscription goes away.

### A caveat: the default receive path changes

Setting `can_loan_messages` true makes **rclcpp's executor route every
subscription through `rmw_take_loaned_message()`** —
`Executor::execute_subscription` branches on `can_loan_messages()`. That changes
the path even for applications that never ask for a loan, so run all four suites
after touching the value.

### Verification

| Check | Result |
|---|---|
| `test_rmw_implementation` | Every test runs, **none skipped**, none failed; the loan tests used to skip |
| A loan round trip ([`test/loaned_messages/`](../test/loaned_messages/)) | Every message sent is received, contents matching |

The round trip uses `test_msgs/UnboundedSequences`, which has strings and
variable-length sequences. A reuse bug that leaks the previous contents does not
show with a fixed-size-only type.

## 23. Line coverage

`./tools/coverage.sh` measures line coverage of `rmw_someip_cpp/src/`. How to
run it is in [`test/README.md`](../test/README.md).

Coverage rose in these steps:

1. The four official suites alone, where measurement started.
2. Unit tests for the pure parts of the library.
3. The SOME/IP abstraction, and unit tests for `context_impl` on top of it.
4. Hand-written descriptors, extern mode and graph queries.
5. The reachable remainder, closed one at a time.
6. Injectable allocation failure.

The unit tests are split between `unit_tests`, for the internals, and
`rmw_api_tests`, for the rmw entry points. The current figures are in the
coverage report CI publishes on every run.

### What measuring revealed

**The official suites only reach the API surface.** `test_rmw_implementation`
covers argument checking thoroughly, but `someip_serializer.cpp` was **not covered at all** —
extern mode runs from a separate compose file, so the suites never execute a
line of it. `extern_mapping.cpp` was barely covered, meaning nothing verified how a
malformed configuration file is rejected. Both are pure functions and unit tests
cover them.

### Changes made for testability

**Rebuild the library instrumented.** The official suites dlopen the `.so`, so
`--coverage` on a test binary counts none of the library's lines
(`-DRMW_SOMEIP_COVERAGE=ON`).

**Compile the internals in, link the public API.** Anonymous namespaces and
header-only classes in `detail/` are unreachable from outside, so the unit tests
compile the sources directly. Doing the same in a test of the rmw C API would
define its symbols twice, so that one is a separate executable linking the `.so`.

**Inject allocation failure through `rcutils_allocator_t`.** rmw takes an
allocator by argument, so a failing one reaches the `goto fail` side from
outside. `FailingAllocator` fails from the nth allocation onward, and varying n
walks the cleanup paths of a partial failure.

**Made configuration file ownership explicit.** `ConfigGenerator::cleanup()` was
inferring "did I create this?" from a `/tmp/rmw_someip_` prefix. A test that
passed a path with that prefix as a user configuration had it deleted, so
`generate()` now returns `Config { path, generated }`. The old behaviour would
delete a user's own file at shutdown if they pointed
`RMW_SOMEIP_CONFIG_FILE` at such a path.

### Bugs this found

**`cdr::Reader` read the header as payload on short input.** With `position_`
before `origin_` (4), `align()`'s subtraction wraps as size_t, slips past the
bounds check and reads encapsulation header bytes as payload.
`deserialize_message()` checks the length first, so nothing was actually broken,
but `Reader` on its own was not defending itself.

**The SOME/IP serializer cannot handle wstrings.** Writing the test is what made
this explicit. The AUTOSAR SOME/IP Transformer has no standard mapping for
UTF-16 strings, and there is no format to agree on with a non-ROS ECU. Failing
is safer than silently emitting a broken encoding, and the test pins that down.

### What was outstanding when the rest of this work started

| Kind | What reaching it would take |
|---|---|
| vsomeip callback paths: the availability and subscription handlers in `context_impl.cpp`, init failure in `someip_session.cpp` | Wrap `vsomeip::application` in an interface and inject a fake. **At the time it depended on the concrete class and could not be substituted** |
| Allocation failure in entity creation, such as `rmw_publisher_allocate()` returning null | Substitute `malloc` rather than an rmw-provided allocator: LD_PRELOAD, or `osrf_testing_tools_cpp`'s memory tools |
| The "vsomeip application is not available" paths | Operate on a context after shutdown; some of these cannot be constructed through the current rmw API |
| Defensive branches for a malformed rosidl descriptor | Inject a hand-written `MessageMembers`; a generated type cannot produce them |
| Everything else: error message construction, hard-to-reach branches | One at a time |

The sections that follow are the record of working through that table.

### Abstracting SOME/IP

The vsomeip abstraction the table called for.
[`detail/someip_endpoint.hpp`](../rmw_someip_cpp/src/detail/someip_endpoint.hpp)
is the seam between the RMW and vsomeip, `SomeipSession` is the production
implementation, and tests inject a `FakeEndpoint`.

That made it possible to test `context_impl.cpp`, the largest file in the
implementation, **without SOME/IP at all**. The line
count was not the point; being able to provoke events that cannot be timed
against the real thing was:

- a peer appearing and disappearing (`fire_availability`)
- a peer's graph arriving: mismatched types, a malformed message, an echo of
  this participant's own guid, an instance_id collision (`fire_message`)
- a late subscription (`fire_subscription`), for TRANSIENT_LOCAL history replay

Changing the arguments from vsomeip types to plain values shortened the call
sites too: an `event_type_e` ternary and the `create_payload()` boilerplate
disappeared from them.

### What is still outstanding

| Kind | What reaching it would take |
|---|---|
| Extern mode on the wire, the SOME/IP-format traffic itself | A non-ROS peer; a unit test gets as far as the descriptor |
| `endpoint() == nullptr` — **unreachable defence** | See below; kept deliberately |
| vsomeip and OS dependencies: startup failure in `someip_session.cpp`, `getifaddrs()` failing, a configuration file that cannot be written, port exhaustion | Substitute `vsomeip::runtime` and `application` themselves, and the syscalls. The seam is one level further out |
| Later failures in `rmw_init()`: creating the SomeipSession, starting the graph channel | As above |
| The `break` arms in `rmw_take_event` — **unreachable defence**, since every `EventData::take()` returns true | As above |
| Argument checks behind other argument checks, such as `rmw_validate_*()` itself failing | Unreachable unless the earlier check is removed |
| Allocation failure where the allocator is fixed, as in `rmw_get_node_names()` | Substituting `malloc`, which is ruled out below |
| String assignment failing, `rosidl_runtime_c__String__assignn()` returning false | As above |
| Everything else, mostly continuation lines gcov attributes to a multi-line expression | — |

### Injecting allocation failure

Neither `rmw_publisher_allocate()` nor `new (std::nothrow)` takes an allocator,
so substituting an `rcutils_allocator_t` cannot reach them. That was the largest
single gap.

**`malloc` itself is not replaced.** That was tried: rosidl's
`get_message_typesupport_handle()` throws from a `noexcept` context when
allocation fails, which ends in `std::terminate`. Another library cannot be made
OOM-tolerant from here.

Instead only the two entry points **specified to return null on failure** are
replaced ([`test/unit/alloc_hooks.cpp`](../test/unit/alloc_hooks.cpp)).

| Replaced | What it catches |
|---|---|
| `operator new(size_t, nothrow)` | `new (std::nothrow) PublisherData()` and friends |
| `rmw_allocate()` | what `rmw_publisher_allocate()` and its siblings call |

**Symbols defined in the executable resolve ahead of a shared library's**, so
allocations inside librmw are caught without `LD_PRELOAD`.

The failure point is thread-local. vsomeip runs several dispatch threads that
allocate constantly, and a process-wide count would make "the nth allocation"
differ from run to run. One failure disarms the hook, since failing indefinitely
would take cleanup and error-message construction down with it.

The tests **do not pin a number**. The failure point is swept from 0 upwards,
and wherever it lands the requirements are the same: do not crash, do not return
a half-built object, set an error when returning null. Between them these walk
every `goto fail` and `delete` path in the creation functions.

That took `rmw_wait.cpp` from 11 uncovered lines to 2, `rmw_node.cpp` from 7 to
2, `rmw_publish.cpp` from 23 to 15, and `rmw_client.cpp` and `rmw_service.cpp`
from 15 to 9.

#### Fixed along the way: MessagePool called std::malloc directly

The loan pool used `std::malloc` and `std::free` directly. This is **rmw-owned
memory** lent to the caller, so it should go through rmw's own
`rmw_allocate()` and `rmw_free()` — which also makes it substitutable from a
test.

#### Why unreachable defences are kept anyway

The lines behind `endpoint() == nullptr` cannot run as the code stands. A
session is always passed to the constructor, since `rmw_init()` does not build a
context if `SomeipSession::create()` failed; it is destroyed only in the
destructor; and `rmw_context_fini()` refuses while nodes remain, so no entity
outlives it.

But **`rmw_shutdown()` used to destroy the session, and it really did crash**:
`rcl_logging_rosout_fini()` removes the rosout publisher after `rmw_shutdown()`,
so a destroyed session was being touched. Moving the destruction point reopens
that hole, so the checks stay and the header comment now describes the actual
invariant.

The `break` arms in `rmw_take_event` are the same kind: all four
`EventData::take()` overloads return `true` unconditionally, so none of them
runs. Dropping the `bool` would remove them, and would silently break the day
`take()` becomes conditional.

### Hand-written introspection descriptors

A descriptor rosidl generates always satisfies certain properties — arrays carry
accessor functions, type_ids are known — so real messages alone never reach the
serializers' defensive branches. But descriptors can be written by hand, which
is exactly what dynamic type support and conversions from other middleware do.
[`test/unit/test_synthetic_members.cpp`](../test/unit/test_synthetic_members.cpp)
builds `MessageMembers` directly and puts them through both serializers.

That took `type_support.cpp` from 45 uncovered lines to 5 and
`someip_serializer.cpp` from 44 to 6. What remains is allocation failure during
string assignment.

#### Bug found: the stride of an accessorless fixed-size array

The fallback that walks an array without accessor functions used
`primitive_size()` as the stride. That is 0 for strings and nested structs, so
**every element pointed at the first one**. All four read and write paths, CDR
and SOME/IP, had it.

rosidl generates accessors for fixed-size arrays too, so a generated type never
takes this path, but a hand-written descriptor does.
`element_stride<Traits>()` returns the element type's real size, and an
undeterminable one — a nested type with no `members_`, say — is an error. The
SOME/IP read path was also missing the check for a dynamic sequence with neither
accessor, which now matches the others.

#### Bug found: a name array that failed to allocate kept its size

`rcutils_string_array_init()` leaves `size` at the requested value when
allocation fails, with only `data` null. `rmw_get_topic_names_and_types()`
passed that straight back, so a caller would loop `size` times over a null
pointer. The output is now zeroed on failure
([`rmw_graph.cpp`](../rmw_someip_cpp/src/rmw_graph.cpp)).

### Unit-testing extern mode

Putting an `ExternMapping` into `rmw_context_impl_s` and injecting a
`FakeEndpoint` covers offering and subscribing with the configuration file's
pinned IDs, with no non-ROS peer — the extern branches in
`context_impl.cpp`.

### Bug found: an unreadable configuration file is silently ignored

Pointing `RMW_SOMEIP_CONFIG_FILE` at a nonexistent path or malformed JSON
changes nothing: **vsomeip ignores it and starts with defaults**. Measured with
a nonexistent path, invalid JSON and an empty `{}` — `rmw_init()` succeeded in
all three. A mistyped path produces no diagnostic and the process runs with
settings nobody chose. `ConfigGenerator::generate()` now checks the file is
readable and fails `rmw_init()` if it is not.

### Removed: `ExternMapping::subscribed_topics()`

Nothing in the library called it; only a test did. An external service's
endpoints are learned from SOME/IP-SD, so a subscribed topic contributes nothing
to the configuration JSON. It existed only for symmetry with `offered_topics()`.

## 24. Benchmark

`performance_test` is not packaged for apt and its dependencies are heavy. What
a comparison needs is one-way latency and loss, so
[`test/benchmark`](../test/benchmark) measures exactly that.

```bash
./tools/ci/in-docker.sh benchmark
BENCH_SIZES="64 65536" BENCH_RMWS=rmw_someip_cpp ./tools/ci/in-docker.sh benchmark
```

Publisher and subscriber are **separate processes**. In one process the traffic
falls into `deliver_locally()` and never touches SOME/IP, which would measure
the wrong thing. Being on one host they share `steady_clock`, so the difference
from the send time in the message is the latency.

**No means are reported.** The differences between implementations show in the
tail, so this reports p50, p90, p99 and the maximum. The first samples are
discarded, since a cold TCP congestion window and page faults raise even the p50.

### Bug found: primitive arrays were written one element at a time

The first measurement showed **latency exactly linear in payload size**, which
made a single large message far slower than a network round trip.

The cause was the condition on the CDR serializer's bulk-copy path.

```cpp
if (is_fixed && width != 0 && member.get_const_function == nullptr) { ... }
```

**rosidl generates accessor functions for fixed-size arrays as well as
sequences**, so `get_const_function == nullptr` never holds for a generated
type. The fast path had never once run, and a `uint8[]` was calling
`write_single()` once per byte. The clue was that line showing as uncovered.

Whether the elements are contiguous follows not from the absence of accessors
but from **`get_const_function` being non-null**: rosidl leaves it null only for
types that are not contiguous — bit-packed `std::vector<bool>` — and supplies
`fetch_function` there instead. So the pointer `get_const_function(field, 0)`
returns can be taken as the base and `width * count` bytes copied at once.

The SOME/IP serializer has the same shape, but being big endian it is limited to
1-byte elements, where no byte order swap is needed.

After the fix, latency is nearly flat across payload sizes; the largest
payloads improved by more than an order of magnitude.

### Where it stands

Measured as one-way latency between separate processes on one host, across
payload sizes from tens of bytes to tens of kilobytes, with both reliable and
best-effort QoS, against `rmw_fastrtps_cpp`.

No loss and no reordering under any condition.

**Latency is consistently higher than Fast DDS**, at every payload size and with
either reliability. The gap is structural: vsomeip routes
even local traffic through a routing manager process, where Fast DDS hands it
over in shared memory. It is not something the RMW implementation can close.

The gap widens for payloads larger than one datagram, where SOME/IP-TP
segmentation takes effect and a message becomes many segments over UDP.
