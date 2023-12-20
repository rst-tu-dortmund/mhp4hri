FROM osrf/ros:melodic-desktop-full


# Prepare for ROS packages
RUN apt-get update \  
    && apt-get -qq upgrade \
    && apt-get -qq install \
    curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup sources list
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add


# Install dependencies
RUN apt-get -qq update \
    && apt-get -qq upgrade \
    && apt-get -qq install \
    gfortran \
    wget \
    patch \
    gcc \
    g++ \
    git \
    cmake \
    liblapack-dev \
    libmetis-dev \
    pkg-config \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Install MUMPS (as default option, but HSL is recommended but has to be installed manually)
RUN mkdir -p ipopt_project; \
    cd ipopt_project; \
    git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git; \
    cd ThirdParty-Mumps; \
    ./get.Mumps; \
    ./configure; \
    make; \
    make install; \
    ldconfig


# Install IPOPT
RUN cd /ipopt_project; \	
    git clone --branch releases/3.14.12 https://github.com/coin-or/Ipopt.git; \
    cd Ipopt; \    
    mkdir -p build; \
    cd build; \
    ../configure; \
    make -j \
    && make install; \
    ldconfig

# Install MHP
RUN apt update; \
    mkdir -p ~/catkin_ws/src; \	
    cd ~/catkin_ws/src; \
    git clone https://github.com/rst-tu-dortmund/mhp4hri.git --branch melodic-devel-github; \
    cd ..; \
    rosdep init; \
    rosdep update --include-eol-distros; \
    rosdep install --from-paths src --ignore-src -y

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash &&\
    cd ~/catkin_ws &&\
    catkin_make && \
    . /root/catkin_ws/devel/setup.bash'

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
WORKDIR /root/catkin_ws


