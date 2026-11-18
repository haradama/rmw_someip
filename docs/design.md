# rmw_someip design

The design of an implementation that uses SOME/IP (vsomeip) as a ROS 2 RMW.

## Background and goal

ROS 2's `rmw` layer was designed around DDS, but the interface itself is
transport agnostic, as `rmw_zenoh` demonstrates.

In automotive, the AUTOSAR Classic and Adaptive platforms use SOME/IP as their
standard protocol. Integrating ROS 2 with them usually means a bridge node
([ROS2_AUTOSARAP_Bridge_Converter](https://github.com/azu-lab/ROS2_AUTOSARAP_Bridge_Converter)
and similar), which becomes both a bottleneck and a single point of failure, and
which never makes the ROS graph itself SOME/IP native.

**Goal**: put all communication on SOME/IP with no change to the ROS 2
application, from `RMW_IMPLEMENTATION=rmw_someip_cpp` alone, including
communication with a native vsomeip application that has no ROS dependency.

## The central problems

| # | Problem | Answered in |
|---|---|---|
| **C1** | ROS 2 topic names are arbitrary strings; SOME/IP has only 16-bit numeric IDs | [The ID model and the graph channel](#the-id-model-and-the-graph-channel) |
| **C2** | A SOME/IP-SD entry carries numeric IDs, a version and a TTL, with nowhere for a type name, QoS or node name | [The graph channel](#the-graph-channel) |
| **C3** | vsomeip expects a static JSON configuration; ROS 2 creates topics at runtime | [The ID model](#the-id-model) |
| **C4** | SOME/IP has no equivalent of durability, deadline, lifespan or history | [QoS](#qos) |
| **C5** | AUTOSAR SOME/IP Transformer, or CDR wrapped in a payload? | [Serialization](#serialization) |

## vsomeip behaviour the design rests on

The ID model depends on how vsomeip actually behaves, which its documentation
does not fully settle.

### What has to be declared in the JSON

| Subject | Must be declared? | Behaviour |
|---|---|---|
| `service` / `instance` | **Yes** | Without a declaration with a port under `services`, no SD offer is sent. The API returns `OFFER(...) (true)` and **not one packet reaches the wire** |
| `event` / `eventgroup` | **No** | The runtime API alone is enough |
| The consumer's `services` | **No** | Endpoints are learned from SD |

`offer_service()` returning success while sending nothing is invisible without
following the logs or the wire, so everything that offers a service — the
participant itself and extern topics the ROS side offers — is checked for a
port before vsomeip starts.

### Events within one service are dynamic

With one service declared in the JSON, events and eventgroups can be created at
runtime without limit, and a consumer that subscribes to some of them receives
nothing from the others.

**Consequence**: one service per process, and one event plus one eventgroup per
topic, needs a single declaration however many topics are created.

### Wildcard discovery

One `request_service(SERVICE_ID, ANY_INSTANCE)` discovers every participant
offering that service ID, each under its own instance.

**Consequence**: a whole ROS domain fits into **one service ID, with
instance_id identifying the participant**.

> An `ANY_INSTANCE` availability handler also fires once with
> `instance=0xFFFF`. That is not a participant and is ignored.

### One routing manager per host

vsomeip allows exactly one routing manager per host — more precisely, per set
of processes sharing `/tmp`, which it uses for local routing. An application
whose configuration names itself as `"routing"` fails `init()` if another
routing manager is already present. With `"routing"` omitted, the first
application to start becomes the routing manager and the rest become proxies.

### The UDP message size limit

A SOME/IP message over UDP has a maximum length, 1416 bytes by default.
Anything larger is dropped at the sender (`Dropping to big message`) unless
SOME/IP-TP segmentation is configured, and vsomeip's TP configuration lists
event IDs individually.

### Container networking

| Symptom | Cause | Fix |
|---|---|---|
| Discovery never starts (`netlink: ... mc=0`) | The container has no `224.0.0.0/4` route, and vsomeip treats a multicast route's presence as the signal that the network is ready | `NET_ADMIN`, plus `ip route add 224.0.0.0/4 dev eth0` at startup |

Multicast forwarding across the Docker bridge itself works with
`multicast_snooping=1` unchanged. vsomeip's `unicast` has to match the
container's IP, so compose files assign static addresses.

## Architecture

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
`vsomeip::application`, one SOME/IP instance. Processes on one host share a
routing manager; under Docker each container is its own network namespace with
its own `/tmp`, so each has its own.

**The seam to vsomeip**: the RMW reaches vsomeip only through
[`detail/someip_endpoint.hpp`](../rmw_someip_cpp/src/detail/someip_endpoint.hpp),
an interface over plain values rather than vsomeip types. `SomeipSession` is
the production implementation; unit tests inject a fake.

## The ID model and the graph channel

### The ID model

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

**Ports are reserved, not derived.** Every participant in a network namespace
needs distinct ports, and a collision leaves one vsomeip unable to bind and so
unable to communicate at all. A participant holds a `flock` on a per-port file
under `/tmp` for the life of the process. `/tmp` is shared with exactly the
processes a port can collide with, so this excludes other rmw_someip processes
reliably even when many start at once. A bind probe is kept as well, to catch
ports held by something that is not rmw_someip; on its own it is not enough,
because the probe socket is closed before vsomeip binds.

**An instance_id collision is detected, not repaired.** Sixteen bits can
collide. On one host that is very unlikely. Across hosts vsomeip catches it
during discovery and logs `Rejecting routing info ... offered previously on
this node`, and no graph message arrives. The graph channel also carries a
64-bit participant fingerprint (guid), so "same instance_id, different guid"
is reported as a collision rather than mistaken for this participant's own
echo and dropped in silence. There is no automatic renumbering: the instance
has already been offered, and redoing discovery would lose more than the
collision does. Splitting by domain or by host avoids it.

### The graph channel

Every participant publishes its own slice of the ROS graph as a SOME/IP
**field** on event `0x8000` / eventgroup `0x0001` of its own service.

A subscriber discovers every participant with one
`request_service(SERVICE_ID, ANY_INSTANCE)` and subscribes to eventgroup
`0x0001` on each instance.

**Using a field is what matters here**: a field delivers its latest value on
subscribe, so a participant that starts later still receives the current graph.
That gives the equivalent of DDS TRANSIENT_LOCAL at the discovery layer.

**The channel is reliable (TCP).** Every node carries six parameter services
plus `get_type_description`, so the graph outgrows one UDP datagram with only a
few nodes, and a dropped graph means a peer is never discovered.

The payload is a compact binary format
([`detail/graph_message.hpp`](../rmw_someip_cpp/src/detail/graph_message.hpp)).
In outline:

```
magic "RSGI" / version / participant instance_id / participant guid
nodes      -> { namespace, name, enclave }
endpoints  -> {
    kind (pub/sub/service/client)
    node_namespace, node_name
    topic_name          # fully qualified ROS name
    type_name           # "std_msgs/msg/String"
    type_hash           # REP-2011 (RIHS)
    event_id            # the value actually assigned; authoritative
    eventgroup_id
    method_id           # services and clients only
    QoS: reliability, durability, history, depth, liveliness,
         deadline, lifespan, liveliness lease duration
    gid
}
```

- **Nodes are listed separately** from endpoints, so a node with no entity
  still appears in `ros2 node list`.
- **The complete QoS is carried**, not only the reliability and durability
  that compatibility depends on. `rmw_get_publishers_info_by_topic()` is
  contractually required to return the actual QoS, resolving BEST_AVAILABLE
  reads it, and the incompatible-QoS events name the offending policy.
- **The type hash** comes from the outer typesupport handle rcl passes, falling
  back to the inner introspection handle. Typesupports without one exist, so
  failing to find it is not an error.
- **Type names are normalized.** The C introspection spells a namespace
  `"std_msgs__msg"` and the C++ one `"std_msgs::msg"`; both become
  `"std_msgs/msg"`, or rclcpp and rclpy nodes would not agree on a type.

This is a private format rather than a rosidl message package in the manner of
`rmw_dds_common`: the channel is internal to the RMW and never reaches a
non-ROS SOME/IP peer, which goes through extern mode's static mapping instead.
The leading magic number and version make a format change detectable.

SOME/IP-SD therefore carries only that a participant exists and how to reach
it, and everything ROS-specific travels on the graph channel. All graph
introspection (`rmw_get_node_names`, `rmw_get_topic_names_and_types`,
`rmw_count_*`, the `*_by_node` and `*_info_by_topic` queries) is answered from
the `GraphCache` built from it. `no_demangle` makes no difference: it is a DDS
notion, and ROS names are carried verbatim.

### Generating the vsomeip configuration

`rmw_init()` writes a vsomeip JSON from the ROS environment (`ROS_DOMAIN_ID` and
friends), points `VSOMEIP_CONFIGURATION` at it, and then calls
`create_application()`. It contains only what the ID model marks as needing a
declaration, plus the SD parameters and the following.

- **Domain separation**: the SD multicast port is offset by
  `30490 + domain_id * 2`, and the service ID carries the domain too.
- **Routing**: `"routing"` is omitted by default, so several ROS processes on
  one host work without coordination. A setup with a dedicated routing daemon
  names it in `RMW_SOMEIP_ROUTING_HOST`.
- **SOME/IP-TP**: topic event IDs are sequential from `0x8001`, so a fixed
  number of them is declared for segmentation up front — 256 by default,
  changed through `RMW_SOMEIP_TP_EVENT_COUNT`. BEST_EFFORT messages far larger
  than one datagram, such as PointCloud2, then get through.
- **Extern topics** the ROS side offers add their own service entries.

`RMW_SOMEIP_CONFIG_FILE` is the escape hatch for a hand-written configuration.
vsomeip silently falls back to defaults when the file it is pointed at is
missing or malformed, so the RMW checks that the file is readable and fails
`rmw_init()` otherwise. A user-supplied file is never deleted; only a generated
one is removed at shutdown.

### On the wire

A `std_msgs/msg/String` published in native mode, as captured on the bridge
between two containers:

```
f000 8002    Message ID: service 0xF000, event 0x8002
0000 0022    Length 34
0000 0010    Request ID: client 0x0000, session 0x0010
01 00 02 00  ProtoVer 1, IfaceVer 0, MsgType 0x02 NOTIFICATION, RetCode E_OK
0001 0000    payload begins: CDR little-endian encapsulation header
1200 0000    CDR string length 18, including the NUL
"Hello SOME/IP: 14\0"
```

The service ID is `0xF000 + domain_id`, and event `0x8002` is the second
sequential assignment; `0x8001` went to the rosout publisher. The CDR payload
sits directly inside the SOME/IP NOTIFICATION frame.

## Native and extern mode

Talking to a non-ROS SOME/IP application means its IDs and encoding are fixed
by the other side, so topics come in two kinds.

| | **Native** | **Extern** |
|---|---|---|
| For | ROS to ROS | ROS to a non-ROS SOME/IP peer, such as an AUTOSAR ECU |
| IDs | assigned sequentially; the graph channel is authoritative | pinned in `someip_mapping.yaml` |
| Serialization | CDR | AUTOSAR SOME/IP Transformer |
| Peer joins the graph channel | required | not required; the RMW synthesizes the graph entry |

Extern mode is effectively a bridge built into the RMW, which removes the need
for a bridge node in its own process. The ROS 2 source never mentions SOME/IP.

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
the participant's, so a topic the ROS side offers gains its own entry in the
generated JSON and must declare a port. A subscribed topic contributes nothing
to the JSON: the peer's endpoints are learned from SD.

[demo/](../demo/) runs both directions against a native vsomeip application.

## QoS

| ROS 2 QoS | Mapping | Implemented in |
|---|---|---|
| `RELIABLE` | TCP endpoint, the `reliable` port | vsomeip configuration |
| `BEST_EFFORT` | UDP endpoint, the `unreliable` port | vsomeip configuration |
| `TRANSIENT_LOCAL`, depth 1 | **offered as `ET_FIELD`**, which delivers the latest value on subscribe | vsomeip, natively |
| `TRANSIENT_LOCAL`, depth above 1 | `ET_EVENT`, a send history, and a replay to each new subscriber | the RMW |
| `KEEP_LAST(depth)` | receive queue bounded to depth | the RMW |
| `KEEP_ALL` | unbounded queue | the RMW |
| `DEADLINE` | send and receive intervals watched on the steady clock | the RMW, in a watchdog |
| `LIFESPAN` | expired messages dropped from the receive queue | the RMW |
| `LIVELINESS: AUTOMATIC` | the graph channel and SD availability count as alive | SD and the RMW |
| `LIVELINESS: MANUAL_BY_TOPIC` | the publisher watches its lease; the subscriber judges from arrival gaps | the RMW |

**Compatibility is the RMW's job.** Unlike DDS, SOME/IP does not refuse to
connect on mismatched QoS. The RMW checks compatibility itself against the
graph cache and does not `subscribe()` to an incompatible peer; without that, a
BEST_EFFORT publisher and a RELIABLE subscriber would silently connect. One
check, in [`detail/qos_compat.hpp`](../rmw_someip_cpp/src/detail/qos_compat.hpp),
backs the public `rmw_qos_profile_check_compatible()`, the matched counts and
the incompatible-QoS events, so the three always reach the same conclusion. It
reports the offending policy, which the events need, and it warns about
SYSTEM_DEFAULT liveliness as well as reliability and durability, as other
implementations do.

**Resolving BEST_AVAILABLE is also the RMW's job**, not rcl's. The shared
utility in `rmw_dds_common`
(`qos_profile_get_best_available_for_topic_{publisher,subscription}`) is applied
to the peers' QoS from the graph channel, so `*_get_actual_qos()` and
`*_info_by_topic()` return concrete values.

### QoS events

SOME/IP has nothing like the DDS StatusCondition, so every event
`rmw_event_type_is_supported()` reports is maintained by the RMW itself.

| Event | Derived from |
|---|---|
| `PUBLICATION_MATCHED` / `SUBSCRIPTION_MATCHED` | graph channel changes, counting only QoS-compatible peers |
| `LIVELINESS_CHANGED` | under AUTOMATIC, a matched publisher is a live publisher; under MANUAL_BY_TOPIC, the subscriber times arrivals |
| `LIVELINESS_LOST` | always 0 under AUTOMATIC, since a running process cannot lose it; under MANUAL_BY_TOPIC, the watchdog sees the lease expire |
| `OFFERED_DEADLINE_MISSED` / `REQUESTED_DEADLINE_MISSED` | the QoS watchdog, ticking every 20 ms |
| `OFFERED_/REQUESTED_QOS_INCOMPATIBLE` | the compatibility check above, counting the peers it rejects and recording `last_policy_kind` |
| `MESSAGE_LOST` | [unsupported](#message_lost-is-not-supported) |

**An event is a cumulative status, not a queue.** `rmw_take_event()` does not
pop one queued event. Like a DDS StatusCondition it returns the running total
plus the delta since the last read and clears the delta, so `taken` is true
even when nothing changed. `event_info` is a different struct per event kind,
and the caller allocates room only for the kind it asked about, so the write is
dispatched on `rmw_event_t::event_type`.

**A deadline** detects that nothing happened for a while, which no
message-driven path can notice; it takes a thread watching the clock (the QoS
watchdog in `rmw_context_impl_s`). It starts only once an entity that needs it
exists, so the common case adds no thread at all.

- The clock starts at **the first write or receipt**. Counting from entity
  creation would report a missed deadline whenever discovery took longer than
  the deadline itself. DDS also counts a per-instance deadline from the first
  sample, so this matches.
- A publisher's deadline is independent of the entity being alive:
  `OFFERED_DEADLINE_MISSED` fires while a publisher exists but has stopped
  publishing.
- One watchdog tick fires at most once per entity, so `total_count_change` is
  always 1. An entity more than one period behind restarts from the current
  time rather than catching up.

**Liveliness** can be read off the graph channel only because AUTOMATIC means
"the participant's process is alive": a dead process shows as SD
availability=false, and a destroyed entity as an updated peer graph.
MANUAL_BY_TOPIC turns on time and is handled
[separately](#manual_by_topic-liveliness).

**Lifespan** is measured in DDS from the sample's source timestamp, but these
payloads are plain CDR — AUTOSAR in extern mode — with nowhere to put a send
time. Adding a header would break both CDR compatibility with Fast DDS and byte
compatibility with a non-ROS peer. Elapsed-since-receipt is used instead, and
expired messages are dropped in `rmw_take`. Messages live slightly longer than
they should by the network delay, but a backlog on a slow subscriber expires
correctly, which is the case that matters.

### TRANSIENT_LOCAL above depth 1

**A vsomeip field holds one value.** It has no notion of history, so it cannot
express a depth above 1. Those topics are offered as `ET_EVENT` instead, with
the RMW keeping a send history, detecting new subscribers through
`register_subscription_handler`, and replaying to that subscriber alone with
`notify_one()`, oldest first.

Sender and subscriber have to agree on the event type, so the decision lives in
one function, `event_type_for(durability, depth)`. The subscribing side reads
**the publisher's** durability and depth from the graph channel, not its own.

The replay does not happen inside the subscription handler. The handler is
called from a vsomeip thread holding vsomeip's own lock, so it only queues a
request, and the QoS watchdog sends.

**Neither fields nor replay reach the same process.** Delivery within a process
goes through `deliver_locally()`, which never touches SOME/IP and has no notion
of history. `deliver_local_history()` therefore delivers the history when a
subscription is created in the publisher's own process, which is why the
history is kept even at depth 1.

### MANUAL_BY_TOPIC liveliness

The semantics are that writing to the topic asserts liveliness. So:

- **The publisher** extends its lease on `rmw_publish` and
  `rmw_publisher_assert_liveliness()`, and raises `LIVELINESS_LOST` when it
  expires.
- **The subscriber** decides from whether anything arrived from that publisher
  within the lease, which adds nothing to the wire.

Because subscribers decide from arrivals, an assertion that sent nothing would
still be reported not alive. So `rmw_publisher_assert_liveliness()` **sends a
zero-length notification**. CDR always begins with a 4-byte encapsulation
header, so no ordinary native-mode message is empty and nothing collides.

> In extern mode zero length is a legitimate payload: the AUTOSAR encoding of a
> message with no fields is zero bytes. A non-ROS peer knows nothing about
> liveliness assertions either, so extern subscriptions skip this reading.

**`LIVELINESS_LOST` fires once, when it is lost.** A deadline counts one miss
per period, but liveliness does not keep incrementing while it stays lost;
`DeadlineTracker::set_one_shot()` suppresses it until the next write.

**The alive count is computed in one place.** AUTOMATIC follows graph changes
and MANUAL_BY_TOPIC follows time; writing both into `LIVELINESS_CHANGED`
separately makes them fight and emits spurious events with zero deltas.
`check_subscription_liveliness()` computes both together.

### MESSAGE_LOST is not supported

Detecting a lost message needs a sequence number from the sender. The session
ID in the SOME/IP header is per (service, instance) and increments across
**all** of that instance's events, so it says nothing about one topic. A
sequence number of our own in the payload would work, but that changes the CDR
bytes and breaks both interoperability with `rmw_fastrtps_cpp` and byte
compatibility in extern mode.

## Serialization

The serializer is abstracted with two implementations. Both walk the
`rosidl_typesupport_introspection_c/cpp` `MessageMembers` at runtime, the same
approach as `rmw_cyclonedds_cpp`: no per-type code generation, so every ROS
message works. The traversal is shared (`detail/introspection_traits.hpp`); the
encoding rules differ enough that the implementations are separate.

| | CDR | SOME/IP |
|---|---|---|
| Byte order | little endian | **big endian** |
| Alignment | to the type width, padded | **packed, no padding** |
| Encapsulation header | 4 bytes | **none** |
| Dynamic array length | **element count** (uint32) | **byte count** (uint32) |

### CDR

Used in native mode. `rmw_get_serialization_format()` returns `"cdr"`, which
keeps rosbag2 interoperable, and the bytes are interoperable with
`rmw_fastrtps_cpp` in both directions.

- **Alignment** is measured from after the encapsulation header, not from the
  start of the buffer.
- **A wstring** is a uint32 character count followed by the characters, **each
  4 bytes**. ROS represents wstrings as UTF-16, but
  `rosidl_typesupport_fastrtps_cpp` converts to `std::wstring` before encoding
  and `wchar_t` is 4 bytes on Linux: `u"ab"` becomes
  `02 00 00 00 | 61 00 00 00 | 62 00 00 00`.
- **Primitive arrays and sequences are copied in bulk.** Whether the elements
  are contiguous follows from **`get_const_function` being non-null**: rosidl
  generates accessors for fixed-size arrays as well as sequences, and leaves
  `get_const_function` null only for types that are not contiguous —
  bit-packed `std::vector<bool>` — supplying `fetch_function` and
  `assign_function` there instead. So the pointer `get_const_function(field, 0)`
  returns is the base and `width * count` bytes are copied at once; the field
  pointer itself must not be used, since for a sequence it refers to the
  container. Without the bulk path a `uint8[]` costs one call per byte and
  latency grows linearly with the payload.
- **The C and C++ introspections are both supported** and differ in more than
  layout: string representation and the resize, fetch and assign handling are
  separate code paths. In C, `rosidl_runtime_c__{String,U16String}__assignn()`
  rejects a null pointer, which an empty `std::vector`'s `data()` may be, so an
  empty string is assigned from a non-null pointer.

### SOME/IP native

Follows the AUTOSAR SOME/IP Transformer and is used in extern mode
(`detail/someip_serializer.cpp`).

- With a byte count and no element count, reading anything of variable size
  (strings, nested structs) means skipping over the elements and counting them
  first.
- The bulk-copy path is limited to 1-byte elements, where big endian needs no
  swap.
- **wstrings are rejected.** The Transformer has no standard mapping for UTF-16
  strings, and there is no format to agree on with a non-ROS ECU. Failing is
  safer than silently emitting an encoding nobody else reads.

### Hand-written descriptors

A descriptor rosidl generates always has certain properties — arrays carry
accessor functions, type IDs are known. Hand-written descriptors, which is what
dynamic type support and conversions from other middleware produce, need not.
Both serializers handle an array without accessors by walking it with the
element type's real size (`element_stride<Traits>()`); an element size that
cannot be determined, or a dynamic sequence with no accessor at all, is an
error rather than a guess.

## Mapping the RMW primitives

### Publishers and subscriptions

| RMW | SOME/IP |
|---|---|
| `rmw_create_publisher` | assign an event and eventgroup, `offer_event`, update the graph channel |
| `rmw_publish` | serialize, `create_payload`, `notify()` |
| `rmw_create_subscription` | look the topic up on the graph channel, then `request_event` and `subscribe` on the matching instance |
| `rmw_take` | pop from the receive queue, which the message handler pushes to |
| `rmw_count_publishers` and friends | queries against GraphCache |

**vsomeip subscriptions are refcounted** per (service_id, instance_id,
event_id, eventgroup_id), native and extern alike. Several ROS subscriptions
can share a topic, so only the transition to zero calls `unsubscribe()` and
`release_event()`.

**Same-process delivery bypasses SOME/IP.** A publisher and subscription in one
process are connected by `deliver_locally()`.

With `avoid_ros_namespace_conventions` set, the topic name is a native one and
is not validated against ROS naming rules.

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

**One request can be answered more than once.** rmw does not require exactly
one response per request, and rclcpp's deferred-response pattern calls
`send_response` repeatedly for the same `request_header`. The pending request
is therefore kept after it is answered, in a map capped by dropping the oldest
entries.

Services are not optional: an rclcpp node creates six parameter services and
`~/get_type_description` by default, so no standard node starts without them.
Actions need nothing extra, since `rcl_action` builds them on services and
topics.

### Wait sets

vsomeip invokes its callbacks on its own threads, so `rmw_wait` is built on a
condition variable. Each entity owns a queue and a mutex, and a callback only
**pushes and notifies** — calling user code from inside one would deadlock.

**`RMW_RET_TIMEOUT` is never returned from an infinite wait.** rclcpp's
`GraphListener` waits with `rcl_wait(&wait_set_, -1)`, which reaches
`rmw_wait()` as `wait_timeout = nullptr`, and treats a timeout there as fatal.
The wait set's trigger is a level flag saying somebody woke the set; it does
not record who, so it can outlive its reason: a message that arrives just after
a wake-up is picked up by that scan and drained by the caller, and the next
`rmw_wait()` then returns at once on the leftover flag with nothing to report.
Narrowing that window cannot close it, so a wake-up that finds nothing ready
**waits again on the time that remains**, indefinitely when the wait is
infinite. Detaching the entities and nulling the slots that are not ready
happens once, after the loop.

In `rmw_wait()`, subscriptions, services and clients arrive as each handle's
`data`, the implementation's own type, but **an event arrives as the
`rmw_event_t` handle itself**.

### Loaned messages

The six `rmw_borrow_loaned_message()` functions are implemented and
`can_loan_messages` is true.

**This is not shared-memory transport.** vsomeip's local path is a UNIX domain
socket; CDR still goes over the wire, and the receiver does not read the memory
the sender wrote. The rmw contract is that the caller borrows memory from the
middleware and says nothing about transport, so the contract is met. What is
saved is the application's own allocation and destruction of the message, and
no more. Real zero copy would need a separate shared-memory path for same-host
delivery, which contradicts the premise of putting all communication on
SOME/IP.

The pool is
[`detail/message_pool.hpp`](../rmw_someip_cpp/src/detail/message_pool.hpp). The
introspection's `size_of_`, `init_function` and `fini_function` are enough to
instantiate any type without per-type code.

- A returned message is `fini`ed and `init`ed again. `fini` frees the inner
  buffers of strings and sequences, so reuse only saves the outer allocation,
  but reusing without clearing would leak the previous value into the next
  loan.
- Outstanding loans are tracked in a set. Returning a pointer that was never
  lent is `RMW_RET_ERROR`; letting it through would free memory the pool does
  not own.
- The destructor destroys outstanding loans, so a forgotten return does not
  leak when the publisher or subscription goes away.
- The memory is rmw-owned and goes through `rmw_allocate()` and `rmw_free()`.

> Setting `can_loan_messages` true makes **rclcpp's executor route every
> subscription through `rmw_take_loaned_message()`**
> (`Executor::execute_subscription` branches on it). The receive path changes
> even for applications that never ask for a loan.

### Context lifecycle

**`rmw_shutdown()` only sets a flag; everything is torn down in
`rmw_context_fini()`.** Three things require that:

- rcl keeps destroying entities after shutdown. `rclcpp::shutdown()` calls
  `rmw_shutdown()` and *then* destroys the rosout publisher, and an application
  loop may still publish while rclpy handles a SIGINT. DDS implementations
  likewise keep the participant until fini.
- `rmw_shutdown()` has to return quickly. rclpy's `Context.try_shutdown()` is
  `if ok(): shutdown()`, which races the `rcl_shutdown` in the signal handler;
  work done inside `rmw_shutdown()` widens that window.
- Whether a session exists cannot stand in for the shutdown state:
  `rmw_context_fini()` rejects a context that was not shut down, and node
  creation fails after shutdown.

**The session is destroyed first.** `~rmw_context_impl_s` calls
`session.reset()` explicitly before any other member goes, so no vsomeip
dispatch thread is still running a callback into the graph cache while it is
destroyed. Default reverse-declaration order would destroy the cache first.

**Entities reach the session through a null-safe accessor**, `endpoint()`. As
the code stands the null branch is unreachable: a context is not built if
`SomeipSession::create()` failed, the session is destroyed only in the
destructor, and `rmw_context_fini()` refuses while nodes remain. The checks
stay because moving the destruction point would silently reopen a
use-after-free.

**vsomeip does not own the signal handlers.** An RMW is a library and must not
intercept an application's SIGINT and SIGTERM handling, so the base image
builds vsomeip with `ENABLE_SIGNAL_HANDLING` off.

### Argument checking

Argument checking is part of the rmw contract, down to which error code each
bad argument produces.

- Every function taking a handle verifies its `implementation_identifier`,
  including the *node's* in functions such as `rmw_destroy_publisher()`. A
  handle from another implementation that is accepted leads to a double free
  when the caller retries.
- The code for a null argument is per function: most return
  `RMW_RET_INVALID_ARGUMENT`, but `rmw_service_server_is_available()` and
  `rmw_destroy_wait_set()` return `RMW_RET_ERROR`.
- Output parameters are left in a safe state on failure. For instance
  `rcutils_string_array_init()` leaves `size` at the requested value when its
  allocation fails, so `rmw_get_topic_names_and_types()` zeroes the output
  rather than hand back a null array with a non-zero size.

## Source layout

```
rmw_someip_cpp/src/
├── rmw_init.cpp       # context and vsomeip application lifecycle
├── rmw_node.cpp
├── rmw_publish.cpp, rmw_subscribe.cpp
├── rmw_service.cpp, rmw_client.cpp
├── rmw_wait.cpp, rmw_event.cpp
├── rmw_graph.cpp      # graph introspection
├── rmw_serialize.cpp, rmw_misc.cpp
└── detail/
    ├── someip_endpoint.hpp   # the seam between the RMW and vsomeip
    ├── someip_session.cpp    # its production implementation
    ├── context_impl.cpp      # the graph channel, routing, QoS events
    ├── graph_cache.cpp, graph_message.cpp
    ├── participant.cpp       # IDs and port reservation
    ├── config_generator.cpp  # writes the vsomeip JSON
    ├── type_support.cpp      # the CDR serializer
    ├── someip_serializer.cpp # the AUTOSAR serializer
    ├── extern_mapping.cpp    # someip_mapping.yaml
    ├── entity_data.hpp, event_data.hpp, message_pool.hpp
    └── cdr.hpp, wait_set.hpp, qos_compat.hpp, rmw_checks.hpp
```

## Verification

How to run each of these is in [`test/README.md`](../test/README.md).

| What | Verified by |
|---|---|
| The rmw contract | The official `test_rmw_implementation`, with nothing skipped |
| Every message type, services, actions, rclcpp/rclpy pairs | The official `test_communication`, same-RMW pairs only |
| rclcpp behaviour, signal handling, many nodes | The official `test_rclcpp` |
| Deadline, lifespan, AUTOMATIC liveliness, BEST_AVAILABLE | The official `test_quality_of_service` |
| TRANSIENT_LOCAL above depth 1, MANUAL_BY_TOPIC liveliness, incompatible-QoS events | `test/qos_events` |
| Loaned messages carrying real data | `test/loaned_messages` |
| CDR interoperability with `rmw_fastrtps_cpp` | `test/cdr_compat_test` |
| Internals, extern mode, malformed input, allocation failure | `test/unit` |
| That traffic really is SOME/IP | Capturing the wire (`test/pubsub_smoke`, `demo/`) |

**CDR compatibility is a round trip across implementations**: can Fast DDS read
what this one wrote, and the reverse. Comparing bytes is too strict, because
Fast DDS leaves uninitialized bytes in struct padding, whose contents CDR
leaves unspecified, and reports an over-estimated `buffer_length`.

**Unit tests run without SOME/IP.** A `FakeEndpoint` behind `SomeipEndpoint`
makes it possible to provoke events that cannot be timed against the real
thing: a peer appearing and disappearing, a peer's graph arriving malformed,
with mismatched types, as an echo of this participant's guid or with a
colliding instance_id, and a late subscription for TRANSIENT_LOCAL replay.

## Performance

One-way latency between separate processes on one host, across payload sizes
from tens of bytes to tens of kilobytes, with both reliable and best-effort
QoS, compared with `rmw_fastrtps_cpp` using [`test/benchmark`](../test/benchmark).

- No loss and no reordering under any condition.
- Latency is nearly flat across payload sizes.
- **Latency is consistently higher than Fast DDS**, at every payload size and
  with either reliability. The gap is structural: vsomeip routes even local
  traffic through a routing manager process, where Fast DDS hands it over in
  shared memory. It is not something the RMW implementation can close.
- The gap widens for payloads larger than one datagram, where SOME/IP-TP
  segmentation takes effect and a message becomes many segments over UDP.

## Limitations and risks

| Item | Impact | Status |
|---|---|---|
| `instance_id` collision across hosts | The peer is never discovered | Detected and reported, not repaired. Split by domain or by host to avoid it |
| The routing manager is a single point of failure | All communication on the host stops | Inherent to vsomeip |
| Discovery overhead | SD traffic grows with the number of offers | One service per participant, so offers scale with processes rather than topics |
| The 16-bit event space | Bounds the number of topics per process | Not a practical limit |
| `MESSAGE_LOST` | The event is unsupported | By design |
| wstrings in extern mode | Serialization fails | By design |
| No shared-memory transport | Higher same-host latency than DDS; loans save only allocation | By design |
| Lifespan counts from receipt | Messages live longer than specified by the network delay | By design |
| `test_communication`'s rclcpp/rclpy pairs fail intermittently | A rare test failure at shutdown | See below |

**The rclpy shutdown race.** The intermittent failure is

```
rclpy.try_shutdown() → context.shutdown()
RCLError: failed to shutdown: rcl_shutdown already called on the given context
```

`Context.try_shutdown()` is `if ok(): shutdown()`, and if the signal handler's
`rcl_shutdown` completes between those two calls it throws. That is rcl's
internal state; `rmw_shutdown()` here does nothing but set a flag, so there is
no window to narrow from the RMW side.
