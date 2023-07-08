FROM tiryoh/ros2-desktop-vnc:galactic
LABEL maintainer="Justin Albrecht<justin.albrecht@nist.gov>"

RUN . /opt/ros/galactic/local_setup.sh && \
    apt-get update && \
    mkdir -p ~/ariac_ws/src && \
    cd ~/ariac_ws && \
    git clone https://github.com/usnistgov/ARIAC.git src/ariac && \
    cd ~/ariac_ws/src/ariac && \
    git checkout release_1.6_staging && \
    cd ~/ariac_ws && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src -y -i --rosdistro galactic && \
    apt install openjdk-17-jdk -y && \
    apt install ros-galactic-nav2-simple-commander -y && \
    colcon build

ENV INSIDE_DOCKER ok