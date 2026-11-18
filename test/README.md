# Test packages

## Running everything

The same things CI runs, locally. **The steps live in `tools/ci/*.sh`, and
GitHub Actions runs those scripts in the same image**, so nothing passes here
and fails there.

```bash
./tools/ci/in-docker.sh lint          # ament_lint
./tools/ci/in-docker.sh unit          # unit tests plus CDR byte compatibility
./tools/ci/in-docker.sh conformance   # the four official suites, qos_events, loaned_messages
./tools/ci/in-docker.sh coverage      # line coverage (same as tools/coverage.sh)
./tools/ci/in-docker.sh benchmark     # one-way latency against other RMW implementations
```

The benchmark is the one thing CI does not run; the numbers are too noisy on a
shared runner. Its parameters come from the environment:

```bash
BENCH_SIZES="64 65536" BENCH_RMWS=rmw_someip_cpp BENCH_COUNT=5000 \
  ./tools/ci/in-docker.sh benchmark
```

The official suites are cloned into `.test_deps` and built on first use
(`tools/ci/prepare-test-deps.sh`), then reused.

What the numbers show is in
[docs/design.md](../docs/design.md#performance).

What follows is how to run each test on its own.

## Linters (ament_lint)

`rmw_someip_cpp` enables `ament_lint_common`: copyright, cppcheck, cpplint,
lint_cmake, uncrustify and xmllint. They run from ctest with `BUILD_TESTING=ON`.

```bash
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp/rosuser -v "$PWD":/ws \
  rmw_someip/test:jazzy bash -lc '
  mkdir -p /tmp/rosuser && source /opt/ros/jazzy/setup.bash
  cd /ws && colcon build --packages-select rmw_someip_cpp \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  ctest --test-dir build/rmw_someip_cpp --output-on-failure'
```

`ament_uncrustify --reformat` fixes uncrustify's complaints mechanically; ROS 2
style puts `?` and `:` at the end of a line. cpplint's include order, blank
lines and `else` brace rules have to be fixed by hand.

## cdr_compat_test

Checks that this implementation's CDR output interoperates with a DDS one.

Returning `"cdr"` from `rmw_get_serialization_format()` commits to producing
what other implementations produce; without that neither rosbag2
interoperability nor a DDS bridge works. Both `librmw_someip_cpp.so` and
`librmw_fastrtps_cpp.so` are dlopened in one process and the same message is put
through each `rmw_serialize()`. Values are also round-tripped through this
implementation's own `rmw_deserialize()`.

The types were chosen for the alignment traps they contain: strings (empty,
short, long), nested structs, dynamic sequences, composites with doubles, and a
large uint8 array with a bool.

```bash
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/ws rmw_someip/base:jazzy bash -lc '
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  cmake -S /ws/test/cdr_compat_test -B /ws/test/cdr_compat_test/build -DCMAKE_BUILD_TYPE=Release
  cmake --build /ws/test/cdr_compat_test/build -j
  export LD_LIBRARY_PATH=/ws/install/rmw_someip_cpp/lib:$LD_LIBRARY_PATH
  /ws/test/cdr_compat_test/build/cdr_compat_test
'
```

Expected output:

```
String(empty)                someip=9B fastrtps=9B  OK
String(short)                someip=11B fastrtps=11B  OK
...
=== pass=16 fail=0 ===
```

## pubsub_smoke

A smoke test for the pub/sub path.

An rclcpp Node creates six parameter services and `~/get_type_description` by
default. This test turns them off through `NodeOptions` and a parameter
override, so that it exercises pub/sub on its own.

```bash
./tools/build_in_docker.sh   # build rmw_someip_cpp first
docker compose -f test/pubsub_smoke/compose.yaml up
```

The listener prints `=== SMOKE RESULT: SUCCESS ... ===` once it has received
enough.

### Checking the wire

A ROS log does not show whether the traffic really goes over SOME/IP. Watch the
bridge directly, with the listener still running:

```bash
BR="br-$(docker network inspect rmw-someip-pubsub_net -f '{{.Id}}' | cut -c1-12)"
docker run --rm --network host --privileged rmw_someip/base:jazzy \
  tcpdump -i "$BR" -nn -X 'not port 30490 and not igmp and greater 80'
```

The frame to expect is in
[docs/design.md](../docs/design.md#on-the-wire).

## Standard nodes

An unmodified `demo_nodes_cpp` works, as does the `ros2` CLI.

```bash
docker compose -f test/pubsub_smoke/compose.yaml up
```

Checking by hand:

| Command | Expected |
|---|---|
| `ros2 run demo_nodes_cpp talker` / `listener` | `I heard: [Hello World: N]` |
| `ros2 run demo_nodes_cpp add_two_ints_server` / `add_two_ints_client` | `Result of add_two_ints: 5` |
| `ros2 node list` / `topic list` / `service list` | every participant's entities |
| `ros2 topic info -v /chatter` | type name, QoS, GID and type hash (`RIHS01_...`) |
| `ros2 node info /talker` | its publishers, subscribers and service servers |
| `ros2 topic echo /chatter --once` | rclpy (C introspection) receiving from an rclcpp publisher |
| `ros2 service call /add_two_ints ...` | `AddTwoInts_Response(sum=42)` |

## Official conformance suites

The test packages are not distributed through apt and have to be fetched from
source; `docker/test.Dockerfile` provides their dependencies.

```bash
docker build -f docker/test.Dockerfile -t rmw_someip/test:jazzy .

mkdir -p .test_deps/src && cd .test_deps/src
git clone --depth 1 --branch jazzy https://github.com/ros2/rmw_implementation.git
git clone --depth 1 --branch jazzy https://github.com/ros2/system_tests.git
cd ../..
```

`test_rmw_implementation` comes from the first, `test_communication`,
`test_quality_of_service` and `test_rclcpp` from the second.

```bash
docker run --rm --cap-add NET_ADMIN --user "$(id -u):$(id -g)" -e HOME=/tmp/rosuser \
  -v "$PWD":/ws rmw_someip/test:jazzy bash -lc '
  mkdir -p /tmp/rosuser && ip route add 224.0.0.0/4 dev eth0
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  cd /ws/.test_deps && colcon build --packages-select \
    test_rmw_implementation test_communication test_quality_of_service test_rclcpp
  source install/setup.bash
  export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error
  ctest --test-dir build/test_rmw_implementation -R someip_cpp --timeout 200
  ctest --test-dir build/test_quality_of_service -R someip           --timeout 200
  ctest --test-dir build/test_communication -R rmw_someip_cpp -E rmw_fastrtps_cpp --timeout 150
  ctest --test-dir build/test_rclcpp -R rmw_someip_cpp --timeout 150'
```

HOME has to be writable, because rcl creates a log directory there. Pairs of
different transports (fastrtps against someip) cannot communicate by
construction and are excluded.

`test_deadline` and `test_automatic_liveliness_changed` are **silently skipped**
when `rmw_event_type_is_supported()` returns false. Confirm they actually ran
from the ctest duration (several seconds each) or from `result="completed"` in the
gtest XML.

`test_communication`'s rclcpp/rclpy pairs fail intermittently, on a shutdown
race inside rclpy
([docs/design.md](../docs/design.md#limitations-and-risks)).

**A single clean run does not mean it is stable.** To judge a regression, run
the affected test repeatedly and compare failure rates.

## qos_events

The QoS features the official suites do not cover: TRANSIENT_LOCAL above depth
1, MANUAL_BY_TOPIC liveliness, and incompatible-QoS events. How they work is in
[docs/design.md](../docs/design.md#qos).

```bash
docker run --rm --cap-add NET_ADMIN --user "$(id -u):$(id -g)" -e HOME=/tmp/rosuser \
  -v "$PWD":/ws rmw_someip/test:jazzy bash -lc '
  mkdir -p /tmp/rosuser && ip route add 224.0.0.0/4 dev eth0
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error
  python3 /ws/test/qos_events/local_transient.py
  python3 /ws/test/qos_events/publisher_side.py &
  sleep 1.5
  python3 /ws/test/qos_events/subscriber_side.py
  wait'
```

Expected output:

```
depth=1: late subscribe -> 1 ['1-0'] (expected 1)
depth=5: late subscribe -> 5 ['5-0', ..., '5-4'] (expected 5)
=== LOCAL TRANSIENT RESULT: SUCCESS ===
[pub] LIVELINESS_LOST fired 1 times, total_count=1
[pub] OFFERED_QOS_INCOMPATIBLE fired 1 times, policy=...RMW_QOS_POLICY_RELIABILITY
=== PUB RESULT: SUCCESS ===
[sub] TRANSIENT_LOCAL history received=5 ['history-0', ..., 'history-4']
[sub] LIVELINESS_CHANGED=[(1, 0, 1, 0), (0, 1, -1, 1), (0, 0, 0, -1)]
[sub] REQUESTED_QOS_INCOMPATIBLE fired 1 times, policy=...RMW_QOS_POLICY_RELIABILITY
=== SUB RESULT: SUCCESS ===
```

The three `LIVELINESS_CHANGED` entries are the publisher being discovered, its
lease expiring, and its process exiting. **LIVELINESS_LOST fires exactly once**;
staying lost must not keep incrementing.

## loaned_messages

An end-to-end check of the loaned-message API. The official loaned tests only
cover argument checking and the empty-queue case, so this actually sends data.

Using a type with strings and variable-length sequences
(`test_msgs/UnboundedSequences`) is the point: loans are reused from a pool, so
failing to clear one on return leaks the previous value into the next — which a
fixed-size-only type would never reveal.

```bash
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp/rosuser -v "$PWD":/ws \
  rmw_someip/test:jazzy bash -lc '
  mkdir -p /tmp/rosuser && source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  cd /ws && colcon build --packages-select loaned_messages --base-paths test'

docker run --rm --cap-add NET_ADMIN --user "$(id -u):$(id -g)" -e HOME=/tmp/rosuser \
  -v "$PWD":/ws rmw_someip/test:jazzy bash -lc '
  mkdir -p /tmp/rosuser && ip route add 224.0.0.0/4 dev eth0
  source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_someip_cpp RMW_SOMEIP_LOG_LEVEL=error
  ros2 run loaned_messages loaned_messages'
```

Expected output:

```
can_loan_messages: publisher=true
received 5 of 5, contents match=yes
=== LOANED RESULT: SUCCESS ===
```

> Setting `can_loan_messages` true makes **rclcpp's executor use
> `rmw_take_loaned_message()` for every subscription**, changing the path even
> for applications that never ask for a loan. Run all four suites after touching
> that value.

## Coverage

```bash
./tools/coverage.sh
# Reports: build/coverage.txt (per line), build/coverage.csv, build/coverage/index.html
```

Line coverage of `rmw_someip_cpp/src/`.

**The library itself has to be rebuilt instrumented.** The official suites
dlopen `librmw_someip_cpp.so`, so `--coverage` on a test binary counts none of
its lines (`-DRMW_SOMEIP_COVERAGE=ON`). Optimization is off too, since inlining
removes the lines being counted.

Two things are measured together:

| | What it reaches |
|---|---|
| `test/unit`'s `unit_tests` | The internals of `detail/`, with the sources **compiled straight into the binary**. Anonymous namespaces, header-only classes and the malformed-input branches are unreachable from outside |
| `test/unit`'s `rmw_api_tests` | The public rmw API, linking the installed `.so`; compiling the sources in would define the C API symbols twice |
| The four official suites plus `qos_events` and `loaned_messages` | The real SOME/IP path |

### Provoking an allocation failure

**An API that takes an allocator** — `rmw_get_topic_names_and_types()` and the
like — gets a failing one. `FailingAllocator` in `test_rmw_api.cpp` fails from
its nth allocation onward, and varying n reaches the cleanup paths that only run
on a partial failure, such as `rmw_names_and_types_fini()`.

**An API that does not** — `rmw_publisher_allocate()`, `new (std::nothrow)` —
is out of reach that way. `alloc_hooks.cpp` defines `operator new(nothrow)` and
`rmw_allocate()` in the executable, whose symbols resolve ahead of a shared
library's, so no `LD_PRELOAD` is needed.

`malloc` itself is not replaced: rosidl throws from a `noexcept` context when
allocation fails, which ends in `std::terminate`, and
another library cannot be made OOM-tolerant from here.

### Passing a malformed type descriptor

A `MessageMembers` rosidl generates always satisfies certain properties —
arrays carry accessor functions, type_ids are known — so real messages alone
never reach the serializers' defensive branches. `test_synthetic_members.cpp`
writes descriptors by hand and puts them straight into a `TypeSupportInfo`,
which bypasses the typesupport handle and can therefore have any shape.

### Exercising extern mode on its own

Load a mapping with `ExternMapping::load()`, put it in
`rmw_context_impl_s::extern_mapping` and inject a `FakeEndpoint`
(`ExternContextTest` in `test_context_impl.cpp`). That covers offering and
subscribing with the configuration file's pinned IDs, without a non-ROS peer.
Only the actual SOME/IP-format traffic needs one.

### What is not covered

| Kind | What reaching it would take |
|---|---|
| Extern mode on the wire, the SOME/IP-format traffic itself | A non-ROS peer; a unit test gets as far as the descriptor |
| `endpoint() == nullptr`, and the `break` arms in `rmw_take_event` | Unreachable defences, kept deliberately (see [Context lifecycle](../docs/design.md#context-lifecycle)) |
| vsomeip and OS dependencies: startup failure in `someip_session.cpp`, `getifaddrs()` failing, a configuration file that cannot be written, port exhaustion | Substituting `vsomeip::runtime` and `application` themselves, and the syscalls |
| Later failures in `rmw_init()`: creating the SomeipSession, starting the graph channel | As above |
| Argument checks behind other argument checks, such as `rmw_validate_*()` itself failing | Unreachable unless the earlier check is removed |
| Allocation failure where the allocator is fixed, as in `rmw_get_node_names()`, and string assignment failing | Substituting `malloc`, which is ruled out above |
| Continuation lines gcov attributes to a multi-line expression | — |
