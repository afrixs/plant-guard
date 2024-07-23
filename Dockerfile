ARG ROS_DISTRO=iron
 
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Create Colcon workspace with external dependencies
RUN mkdir -p /dds_ws/
WORKDIR /dds_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update -y
RUN apt-get install -y iproute2
COPY dds_ws/src src
RUN pwd
RUN ls
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --symlink-install

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
RUN pwd
RUN ls
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN source /dds_ws/install/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /dds_ws/install/setup.bash \
 && colcon build --symlink-install --packages-ignore mocked_rpi

RUN mkdir -p /root/plant_guard

EXPOSE 7446/udp
EXPOSE 7447/tcp
EXPOSE 8000/tcp

ENV RUST_LOG info

# Set up the entrypoint
COPY ./docker-entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]