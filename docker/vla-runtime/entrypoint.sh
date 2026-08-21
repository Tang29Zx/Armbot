#!/usr/bin/env bash

set -euo pipefail

if [[ -z "${PC_DDS_IP:-}" ]]; then
    echo "PC_DDS_IP is required (the computer LAN address reachable by RDK)" >&2
    exit 2
fi

python3 - "${PC_DDS_IP}" <<'PY'
import ipaddress
import sys

address = ipaddress.ip_address(sys.argv[1])
if address.version != 4 or address.is_loopback or address.is_unspecified:
    raise SystemExit('PC_DDS_IP must be a non-loopback IPv4 address')
PY

sed "s/__PC_DDS_IP__/${PC_DDS_IP}/g" \
    /etc/armbot/fastdds.pc.xml.in > /tmp/armbot-fastdds.xml

set +u
source /opt/ros/humble/setup.bash
source /opt/armbot_ws/install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-29}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/armbot-fastdds.xml

exec "$@"
