# Reference : https://github.com/changh95/fastcampus_slam_codes/blob/main/Dockerfile
FROM ubuntu:jammy

ENV ROS_DISTRO=humble

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install build-essential -y && \
    apt-get install cmake -y && \
    apt-get install git -y && \
    apt-get install sudo -y && \
    apt-get install wget -y && \
    apt-get install curl -y && \
    apt-get install ninja-build -y && \
    apt-get install software-properties-common -y && \
    apt-get install python3 -y && \
    apt-get install python3-pip -y && \
    apt-get install -y ssh && \
    apt-get install -y gcc && \
    apt-get install -y g++ && \
    apt-get install -y gdb && \
    apt-get install -y cmake && \
    apt-get install -y rsync && \
    apt-get install -y tar && \
    apt-get install -y x11-utils && \
    apt-get install -y x11-apps && \
    apt-get install -y zip && \
    apt-get install -y tzdata && \
    apt-get install -y gnupg2 && \
    apt-get install -y lsb-release && \
    apt-get clean

# ROS2 humble
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-desktop && \
    rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Clang
RUN wget https://apt.llvm.org/llvm.sh && chmod +x llvm.sh && ./llvm.sh 14 && rm llvm.sh
RUN apt-get install clang clangd

# OpenCV
RUN apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
RUN wget https://github.com/opencv/opencv/archive/refs/tags/4.8.1.zip &&\
    unzip 4.8.1.zip &&\
    cd opencv-4.8.1 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    make install &&\
    cd ../../


# Eigen
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.zip &&\
    unzip eigen-3.3.8.zip &&\
    cd eigen-3.3.8 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    make install &&\
    cd ../../


# Sophus
RUN wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.zip &&\
    unzip 1.22.10.zip &&\
    cd Sophus-1.22.10 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    make install && \
    cd ../../


# Pangolin
RUN apt-get install -y mesa-utils && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglu1-mesa-dev && \
    apt-get install -y libglew-dev &&\
    apt-get install -y libglvnd-dev &&\
    apt-get install -y libgl1-mesa-dev &&\
    apt-get install -y libegl1-mesa-dev &&\
    apt-get install -y mesa-common-dev


RUN wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip &&\
    unzip v0.6.zip &&\
    cd Pangolin-0.6 &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    make install &&\
    cd ../../


# Ceres-solver
RUN apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev &&\
    wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip &&\
    unzip 2.1.0.zip &&\
    cd ceres-solver-2.1.0 && \
    mkdir build && cd build &&\
    cmake .. &&\
    make -j4 &&\
    make install &&\
    cd ../../

# glew 2.1
RUN wget https://github.com/nigels-com/glew/releases/download/glew-2.1.0/glew-2.1.0.tgz && \
    tar -xvf glew-2.1.0.tgz && \
    cd glew-2.1.0 && \
    make && \
    make install

RUN apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /root/visual_slam_ws/
