#!/usr/bin/env bash

# Start two URSim containers for the dual-arm ROX configuration.
#
# The 500xx ports configured in rox.urdf.xacro are listening ports on the ROS
# host.  URSim connects to those ports after the driver embeds the selected
# port in the External Control program; they must not be published by Docker.
# Each URSim therefore keeps the standard robot-side ports and is addressed by
# its own IP on the Docker network.

set -euo pipefail

robot_model="ur10e"
arm1_ip="192.168.1.102"
arm2_ip="192.168.1.103"
arm1_name="ursim_arm1"
arm2_name="ursim_arm2"
state_root="${HOME}/.ursim-instances"
ursim_network="ursim_net"
ursim_subnet="192.168.1.0/24"

usage()
{
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -m, --model MODEL       UR robot model for both arms (default: ${robot_model})
      --arm1-ip IP        URSim address for arm 1 (default: ${arm1_ip})
      --arm2-ip IP        URSim address for arm 2 (default: ${arm2_ip})
      --arm1-name NAME    Docker container name for arm 1 (default: ${arm1_name})
      --arm2-name NAME    Docker container name for arm 2 (default: ${arm2_name})
      --state-root PATH   Root for independent URSim state (default: ${state_root})
  -h, --help              Show this help

Both addresses must belong to ${ursim_subnet}. The launcher creates or
validates the Docker network '${ursim_network}' before starting URSim.
EOF
}

while (($#)); do
  case "$1" in
    -m|--model)
      robot_model="$2"
      shift 2
      ;;
    --arm1-ip)
      arm1_ip="$2"
      shift 2
      ;;
    --arm2-ip)
      arm2_ip="$2"
      shift 2
      ;;
    --arm1-name)
      arm1_name="$2"
      shift 2
      ;;
    --arm2-name)
      arm2_name="$2"
      shift 2
      ;;
    --state-root)
      state_root="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$arm1_ip" == "$arm2_ip" ]]; then
  echo "Arm 1 and arm 2 require different URSim IP addresses." >&2
  exit 2
fi

for address in "$arm1_ip" "$arm2_ip"; do
  if [[ ! "$address" =~ ^192\.168\.1\.([1-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-4])$ ]]; then
    echo "Address '$address' is outside the configured URSim subnet ${ursim_subnet}." >&2
    exit 2
  fi
done

mkdir -p "${state_root}/arm1" "${state_root}/arm2"

# start_ursim.sh uses the fixed name 'ursim_net'. Create it with the ROX robot
# subnet before the stock script gets a chance to create its 192.168.56.0/24
# default. Refuse to reuse a network with an incompatible subnet.
if docker network inspect "$ursim_network" >/dev/null 2>&1; then
  configured_subnets="$(
    docker network inspect \
      --format '{{range .IPAM.Config}}{{println .Subnet}}{{end}}' \
      "$ursim_network"
  )"
  if ! grep -Fxq "$ursim_subnet" <<<"$configured_subnets"; then
    echo "Docker network '$ursim_network' already exists with a different subnet:" >&2
    echo "$configured_subnets" >&2
    echo "Stop its containers, remove the network, and run this command again:" >&2
    echo "  docker network rm $ursim_network" >&2
    exit 2
  fi
else
  docker network create --subnet "$ursim_subnet" "$ursim_network" >/dev/null
fi

arm1_started=false
cleanup_failed_start()
{
  if [[ "$arm1_started" == true ]]; then
    docker stop "$arm1_name" >/dev/null 2>&1 || true
  fi
}
trap cleanup_failed_start ERR

# Port forwarding is intentionally disabled. Both containers expose the same
# standard UR ports (29999 and 30001-30004) on different container IPs.
HOME="${state_root}/arm1" ros2 run ur_client_library start_ursim.sh \
  -m "$robot_model" -n "$arm1_name" -i "$arm1_ip" -f DISABLED -d
arm1_started=true

HOME="${state_root}/arm2" ros2 run ur_client_library start_ursim.sh \
  -m "$robot_model" -n "$arm2_name" -i "$arm2_ip" -f DISABLED -d

trap - ERR

cat <<EOF
Started both URSim instances:
  arm 1: ${arm1_name} at ${arm1_ip}
  arm 2: ${arm2_name} at ${arm2_ip}

ROX driver callback ports remain configured by rox.urdf.xacro:
  arm 1: reverse=50001, script_sender=50002, trajectory=50003, script_command=50004
  arm 2: reverse=50006, script_sender=50007, trajectory=50009, script_command=50010
EOF
