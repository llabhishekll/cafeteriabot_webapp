# base image for docker
FROM osrf/ros:humble-desktop-focal

# set environment variable
ENV DEBIAN_FRONTEND=noninteractive

# install core dependency
RUN apt-get update && apt-get install -y \
    git tree nano htop curl iputils-ping \
    python3 python3-pip python-is-python3

# install ros dependency
RUN apt-get install -y \
    ros-humble-rosbridge-server

# setup workspace
RUN git clone -b ros2 https://github.com/llabhishekll/cafeteriabot_project.git /ros2_ws/src/
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; cd /ros2_ws; colcon build"

# add underlay and overlay workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY ./ros_entrypoint.sh /

# set working directory
WORKDIR /ros2_ws

# expose port
# https://docs.ros.org/en/galactic/Concepts/About-Domain-ID.html
EXPOSE 7000 11315 9090 20001

# entry point
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "cafeteriabot_webapp", "cafeteriabot_webapp.launch.py"]