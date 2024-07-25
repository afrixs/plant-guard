#!/bin/bash

sudo ip link set lo multicast on
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /pg_ws/install/setup.bash
# zenoh 0.7.2
#export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery><Internal><SocketReceiveBufferSize min="20MB"/></Internal></Domain></CycloneDDS>'
# zenoh 0.5.0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery><Internal><MinimumSocketReceiveBufferSize>20MB</MinimumSocketReceiveBufferSize></Internal></Domain></CycloneDDS>'
export ROS_DOMAIN_ID=23

# Execute the command passed into this entrypoint
exec "$@"