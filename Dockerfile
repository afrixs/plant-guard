ARG ROS_DISTRO=iron
 
FROM ros:${ROS_DISTRO}-ros-base as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Create Colcon workspace with external dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update -y
RUN apt-get install -y iproute2

#RUN mkdir -p /dds_ws/
#WORKDIR /dds_ws
#COPY dds_ws/src src
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
# && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
#RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
# && MAKEFLAGS=-j1 colcon build --symlink-install --parallel-workers 1
RUN apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-zenoh-bridge-dds cron

# python3-portalocker not working with rosdep :/
RUN apt-get install -y python3-pip \
    && python3 -m pip install --upgrade pip \
    && python3 -m pip install portalocker

FROM base as app
# Create Colcon workspace with external dependencies
RUN mkdir -p /pg_ws/
WORKDIR /pg_ws/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
COPY pg_ws/src src
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN touch src/mocked_rpi/COLCON_IGNORE
RUN touch src/pg_rviz_plugins/COLCON_IGNORE
RUN touch src/bagtube/bagtube_rviz_plugins/COLCON_IGNORE

#RUN source /dds_ws/install/setup.bash \
# && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
#RUN source /dds_ws/install/setup.bash \
# && colcon build --symlink-install --packages-ignore mocked_rpi bagtube_rviz_plugins pg_rviz_plugins
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && MAKEFLAGS=-j1 colcon build --symlink-install --parallel-workers 1
    # && colcon build --symlink-install  # can be used instead in case the RPi has proper cooling maybe...

RUN pip3 install python-crontab RPi.GPIO
RUN echo 'alias r2pg="source /pg_ws/install/setup.bash"' >> ~/.bashrc
# zenoh 0.7.2
#RUN echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>' >> ~/.bashrc
# zenoh 0.5.0
RUN echo "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=23" >> ~/.bashrc

RUN systemctl enable cron

EXPOSE 7446/udp
EXPOSE 7447/tcp
EXPOSE 8000/tcp

ENV RUST_LOG info

# Set up the entrypoint
COPY ./docker-entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]