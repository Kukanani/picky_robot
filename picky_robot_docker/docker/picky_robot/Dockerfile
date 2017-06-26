FROM ubuntu:xenial

RUN apt-get -qq update && \
    apt-get -qq install locales -y

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# upgrade distro
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade -y && \
    apt-get -qq update

RUN apt-get -qq install -y \
      git \
      wget \
      build-essential \
      cppcheck \
      cmake \
      libopencv-dev \
      libpoco-dev \
      libpocofoundation9v5 \
      libpocofoundation9v5-dbg \
      python-empy \
      python3-dev \
      python3-empy \
      python3-nose \
      python3-pip \
      python3-setuptools \
      python3-vcstool \
      libtinyxml-dev \
      libeigen3-dev \
      libasio-dev \
      libtinyxml2-dev

# special depends for ORK renderer
RUN apt-get -qq install -y \
      libboost-dev \
      libassimp-dev \
      freeglut3-dev \
      libgl1-mesa-dev \
      libfreeimage-dev \
      libxmu-dev \
      libxi-dev \
      libsdl1.2-dev \
      libosmesa6-dev \
      libusb-1.0-0-dev \
      libudev-dev

# special depends for Astra camera driver
RUN apt-get -qq install -y \
  libboost-system-dev \
  libboost-thread-dev

# special depends for demo
RUN pip3 install --upgrade pip
RUN pip3 install numpy math3d

# setup ros2 workspace
ENV ROS2_WS=/root/picky_robot_ws
RUN rm -rf $ROS2_WS
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

RUN wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
RUN vcs import src < ros2.repos

# TODO temporary fix for python memory errors
# RUN cd src/ros2/rosidl && git pull && git checkout fix_destroy_segfault
# RUN cd src/ros2/rclpy && git pull && git checkout fix_destroy_segfault

# set up picky robot demos

RUN wget https://raw.githubusercontent.com/Kukanani/picky_robot/ros2/picky_robot.repos
RUN vcs import src/ros2 < picky_robot.repos

# LINEMOD data and meshes
RUN git clone https://github.com/Kukanani/picky_robot_data.git

# TODO temporary fix to avoid dependency on kobuki drivers
RUN touch src/ros2/turtlebot2_demo/turtlebot2_follower/AMENT_IGNORE
RUN touch src/ros2/turtlebot2_demo/turtlebot2_drivers/AMENT_IGNORE

# build everything

WORKDIR $ROS2_WS
RUN src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated --parallel

RUN echo "export ROS2_WS=/root/picky_robot_ws" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc && \
    echo "source $ROS2_WS/install_isolated/local_setup.bash" >> /root/.bashrc


LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}