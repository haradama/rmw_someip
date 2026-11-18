# Phase 0 risk spikes

Minimal vsomeip programs, with no ROS involved, that established whether the
`rmw_someip` design was viable before it was built. The results are written up
in [`../docs/design.md`](../docs/design.md) §3.

| Directory | Question | Answer |
|---|---|---|
| `dynamic_id/` | Can a service and event be offered at runtime without declaring them in the JSON? | The service must be declared; **events and eventgroups can be dynamic** |
| `multi_event/` | Can several topics live in one service, assigned at runtime, and be subscribed to selectively? | **Yes**, with nothing leaking from unsubscribed topics |
| `wildcard_discovery/` | Can `ANY_INSTANCE` discover every participant at once? | **Yes** |

## Running them

From the repository root:

```bash
# Base image: ROS 2 Jazzy plus vsomeip 3.7.4
docker build -f docker/base.Dockerfile -t rmw_someip/base:jazzy .

./tools/build_in_docker.sh spike/dynamic_id spike/multi_event spike/wildcard_discovery

for s in dynamic_id multi_event wildcard_discovery; do
  (cd "spike/$s" && timeout 25 docker compose up; docker compose down -v)
done
```

Each prints `=== SPIKE RESULT: SUCCESS ... ===` when it passes.

## Traps these spikes uncovered

1. **`offer_service()` returns success and sends nothing.** A service not
   declared with a port under `services` in the JSON has its SD offer parked in
   `pending_sd_offers` and never sent. No return value reveals this; only
   tcpdump does.

2. **Service discovery does not start in a Docker container.** vsomeip treats
   the presence of a multicast route (`224.0.0.0/4`) as the signal that the
   network is ready, and a container has none by default. It needs `NET_ADMIN`
   and `ip route add 224.0.0.0/4 dev eth0`; `netlink: ... mc=0` in the startup
   log becoming `mc=1` confirms it. Multicast forwarding across the Docker
   bridge itself works with the default settings.

3. **An `ANY_INSTANCE` availability handler fires first with
   `instance=0xFFFF`.** Implementations have to ignore that one.
