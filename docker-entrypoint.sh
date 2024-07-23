#!/bin/bash

sudo ip link set lo multicast on
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /pg_ws/install/setup.bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery><Internal><SocketReceiveBufferSize min="20MB"/></Internal></Domain></CycloneDDS>'
export ROS_DOMAIN_ID=23

# Execute the command passed into this entrypoint
exec "$@"