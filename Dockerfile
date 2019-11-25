from dorowu/ubuntu-desktop-lxde-vnc:xenial

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-get update -y && \
    apt-get install -y --allow-unauthenticated ros-kinetic-desktop-full ros-kinetic-navigation ros-kinetic-roscpp ros-kinetic-joy ros-kinetic-kobuki-safety-controller ros-kinetic-yocs-velocity-smoother ros-kinetic-geometry-msgs ros-kinetic-yocs-cmd-vel-mux ros-kinetic-diagnostic-aggregator ros-kinetic-depthimage-to-laserscan ros-kinetic-gazebo-ros ros-kinetic-kobuki-gazebo-plugins ros-kinetic-robot-pose-ekf ros-kinetic-robot-state-publisher ros-kinetic-geodesy ros-kinetic-hector-gazebo ros-kinetic-rosbridge* ros-kinetic-opencv-apps ros-kinetic-tf2-geometry-msgs ros-kinetic-web-video-server ros-kinetic-xacro && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

# Upgrade gazebo to gazebo v7
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN curl -s http://packages.osrfoundation.org/gazebo.key | apt-key add -
RUN apt-get update -y && apt-get install -y gazebo7

# OpenUAV 
RUN apt-get update && apt-get install -y --allow-unauthenticated --no-install-recommends \
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
RUN pip install --upgrade pip

run pip install numpy toml opencv-python

run bash -c "source /opt/ros/kinetic/setup.bash; mkdir -p /home/ubuntu/catkin_ws/src; cd ~/catkin_ws/; catkin_make"

#run bash -c "mkdir -p /home/ubuntu/catkin_ws/src && wstool init /home/ubuntu/catkin_ws/src && \
#             rosinstall_generator --rosdistro kinetic --upstream-development mavros | tee /tmp/mavros.rosinstall && \
#             rosinstall_generator --rosdistro kinetic mavlink | tee -a /tmp/mavros.rosinstall && \
#             wstool merge -t src /tmp/mavros.rosinstall && \
#             wstool update -t src && \
#             rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --as-root apt:false"

workdir /home/ubuntu/catkin_ws/src

run git clone https://github.com/0xFORDCOMMA/571-proj.git

run git clone --recursive https://github.com/auman66/Firmware.git

run ln -s $(pwd)/Firmware/Tools/sitl_gazebo ./ && touch Firmware/CATKIN_IGNORE

run bash -c "source /home/ubuntu/.bashrc; source /home/ubuntu/catkin_ws/devel/setup.bash; cd Firmware/Tools/sitl_gazebo; mkdir build && cd build; cmake -D CMAKE_PREFIX_PATH=/opt/ros/kinetic/share/OpenCV-3.3.1-dev -D _MAVLINK_INCLUDE_DIR=/home/ubuntu/catkin_ws/src/Firmware/mavlink/include -D GSTREAMER_LIBRARIES=/usr/lib/gstreamer-1.0 .. && make"

workdir /home/ubuntu/catkin_ws/

run rm src/Firmware/CATKIN_IGNORE

run apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

run bash -c "source /root/.bashrc; wstool init ~/catkin_ws/src; source /home/ubuntu/catkin_ws/devel/setup.bash; apt-get update; rosdep install -i -y --from-paths /home/ubuntu/catkin_ws/src"

workdir /home/ubuntu/catkin_ws/src/Firmware

run make px4_sitl

run bash -c ". /home/ubuntu/catkin_ws/devel/setup.bash && catkin build"
#run bash -c "source /home/ubuntu/.bashrc; source /home/ubuntu/catkin_ws/devel/setup.bash && catkin init && catkin clean --yes && catkin build"

workdir /home/ubuntu

run curl -s https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash

run mkdir src

run git clone https://github.com/dronekit/dronekit-python.git src/dronekit-python

workdir /home/ubuntu/src/dronekit-python/

run python setup.py build && python setup.py install

run echo 'export GAZEBO_PLUGIN_PATH="/home/ubuntu/catkin_ws/src/Firmware/Tools/sitl_gazebo/build/"' | tee -a /home/ubuntu/.bashrc /root/.bashrc
run echo '. /home/ubuntu/catkin_ws/devel/setup.bash' | tee -a /home/ubuntu/.bashrc /root/.bashrc
