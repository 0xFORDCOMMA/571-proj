#!/bin/bash

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-get update -y && \
    apt-get install -y --allow-unauthenticated ros-kinetic-desktop-full ros-kinetic-navigation ros-kinetic-roscpp ros-kinetic-joy ros-kinetic-kobuki-safety-controller ros-kinetic-yocs-velocity-smoother ros-kinetic-geometry-msgs ros-kinetic-yocs-cmd-vel-mux ros-kinetic-diagnostic-aggregator ros-kinetic-depthimage-to-laserscan ros-kinetic-gazebo-ros ros-kinetic-kobuki-gazebo-plugins ros-kinetic-robot-pose-ekf ros-kinetic-robot-state-publisher ros-kinetic-geodesy ros-kinetic-hector-gazebo ros-kinetic-rosbridge* ros-kinetic-opencv-apps ros-kinetic-tf2-geometry-msgs ros-kinetic-web-video-server ros-kinetic-xacro && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros/kinetic/setup.bash" >> /home/ubuntu/.bashrc

# Upgrade gazebo to gazebo v7
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
curl -s http://packages.osrfoundation.org/gazebo.key | apt-key add -
apt-get update -y && apt-get install -y gazebo7

# OpenUAV 
apt-get update && apt-get install -y --allow-unauthenticated --no-install-recommends \
        apt-utils \
        git \
        build-essential \
        curl \
        geographiclib-tools \
        libcurl3-dev \
        libfreetype6-dev \
        libpng12-dev \
        libzmq3-dev \
        pkg-config \
        python-dev \
        rsync \
        software-properties-common \
        unzip \
        zip \
        zlib1g-dev \
        openjdk-8-jdk \
        openjdk-8-jre-headless \
        uwsgi \
        uwsgi-src \
        uwsgi-plugin-python3 \
        python-pip \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        build-essential \
        git \
        curl \
        wget \
        python-catkin-tools \
        tmux \
        $(apt-cache --names-only search ^gstreamer1.0-* | awk '{ print $1 }' | grep -v gstreamer1.0-hybris) \
        $(apt-cache --names-only search ^libgstreamer1.0-* | awk '{ print $1 }' | grep -v gstreamer1.0-hybris) \
        protobuf-compiler \
        libignition-math2-dev \
        libxslt-dev \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
pip install --upgrade pip

pip install numpy toml opencv-python

bash -c "source /opt/ros/kinetic/setup.bash; mkdir -p /home/ubuntu/catkin_ws/src; cd ~/catkin_ws/; catkin_make"

cd /home/ubuntu/catkin_ws/src

git clone https://github.com/0xFORDCOMMA/571-proj.git

git clone --recursive https://github.com/auman66/Firmware.git

ln -s $(pwd)/Firmware/Tools/sitl_gazebo ./ && touch Firmware/CATKIN_IGNORE

bash -c "source /home/ubuntu/.bashrc; source /home/ubuntu/catkin_ws/devel/setup.bash; cd Firmware/Tools/sitl_gazebo; mkdir build && cd build; cmake -D CMAKE_PREFIX_PATH=/opt/ros/kinetic/share/OpenCV-3.3.1-dev -D _MAVLINK_INCLUDE_DIR=/home/ubuntu/catkin_ws/src/Firmware/mavlink/include -D GSTREAMER_LIBRARIES=/usr/lib/gstreamer-1.0 .. && make"

cd /home/ubuntu/catkin_ws/

rm src/Firmware/CATKIN_IGNORE

apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

bash -c "source /root/.bashrc; wstool init ~/catkin_ws/src; source /home/ubuntu/catkin_ws/devel/setup.bash; apt-get update; rosdep install -i -y --from-paths /home/ubuntu/catkin_ws/src"

cd /home/ubuntu/catkin_ws/src/Firmware

make px4_sitl

bash -c ". /home/ubuntu/catkin_ws/devel/setup.bash && catkin build"

cd /home/ubuntu

curl -s https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

mkdir src

git clone https://github.com/dronekit/dronekit-python.git src/dronekit-python

cd /home/ubuntu/src/dronekit-python/

python setup.py build && python setup.py install

echo 'export GAZEBO_PLUGIN_PATH="/home/ubuntu/catkin_ws/src/Firmware/Tools/sitl_gazebo/build/"' | tee -a /home/ubuntu/.bashrc /root/.bashrc
echo '. /home/ubuntu/catkin_ws/devel/setup.bash' | tee -a /home/ubuntu/.bashrc /root/.bashrc
