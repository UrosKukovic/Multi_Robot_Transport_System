FROM ros:lyrical-ros-base

WORKDIR /ws

COPY src/mrts_bringup/package.xml src/mrts_bringup/
COPY src/mrts_fleet_manager/package.xml src/mrts_fleet_manager/
COPY src/mrts_interfaces/package.xml src/mrts_interfaces/

RUN apt-get update && \
    rosdep install -y --from-paths src --ignore-packages-from-source && \
    rm -rf /var/lib/apt/lists/* 

COPY src/ src/
RUN . /opt/ros/lyrical/setup.sh && colcon build

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
