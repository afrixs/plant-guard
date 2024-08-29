sudo ip link set lo multicast on
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
export ROS_DOMAIN_ID=0
(r2pg && ros2 launch plantguard_bringup pump_crane_client.launch.py)