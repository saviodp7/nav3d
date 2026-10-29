FROM px4io/px4-dev:v1.16.2

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DISTRO=jazzy

WORKDIR /root

RUN mkdir -p /ros2_ws/src /ros2_ws/build /ros2_ws/install /ros2_ws/log

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    git \
    locales \
    nano \
    python-is-python3 \
    python3-pip \
    python3-venv \
    software-properties-common \
    sudo \
    wget \
    lsb-release \
    ca-certificates \
    clang-format \
    clang-tidy \
    libegl1 \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libvulkan1 \
    mesa-utils \
    vulkan-tools \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list \
    && wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-dev-tools \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-geographic-msgs \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-moveit \
    ros-jazzy-moveit-hybrid-planning \
    ros-jazzy-moveit-ros-occupancy-map-monitor \
    ros-jazzy-moveit-ros-perception \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-octomap-server \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libfcl-dev \
    libgz-sim8-dev \
    libgz-msgs10-dev \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/nav3d-tools \
    && /opt/nav3d-tools/bin/pip install --no-cache-dir \
    vcstool \
    empy==3.3.4 \
    pyros-genmsg \
    black \
    isort \
    ruff \
    cmakelang

ENV PATH="/opt/nav3d-tools/bin:${PATH}"

RUN git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && cd /root \
    && rm -rf /root/Micro-XRCE-DDS-Agent

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

RUN git config --global --add safe.directory '*'

CMD ["bash"]
