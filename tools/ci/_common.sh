# Shared preamble for the CI scripts, which run inside the container.
#
# The same scripts run in the same image whether invoked from
# tools/ci/in-docker.sh locally or from GitHub Actions, so nothing passes in one
# place and fails in the other. Do not move any of these steps into the
# workflow YAML.
# set -u is unusable: ROS's setup.bash reads undefined variables.
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="${WS:-/ws}"

# rcl creates a log directory, so HOME has to be writable.
export HOME="${HOME:-/tmp/rosuser}"
mkdir -p "$HOME"

# SOME/IP-SD uses multicast (224.244.224.245:30490) and the default container
# has no route for 224.0.0.0/4. Without NET_ADMIN this quietly does nothing;
# the unit tests do not need multicast.
ip route add 224.0.0.0/4 dev eth0 2>/dev/null || true

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "$WS"

log() { printf '\n=== %s ===\n' "$*"; }
