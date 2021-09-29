Bootstrap: docker
From: ubuntu:20.04

%help
  This is a container with Open3D, PCL, and cupoch installed.
  cuda-pcl cannot be installed because it's a binary release only supporting the Jetson.
  The purpose is to compare GPU/CPU implementation of ICP.

%labels
  MAINTAINER Shane Loretz <sloretz@openrobotics.org>

%post
  export DEBIAN_FRONTEND=noninteractive

  if ! which wget
  then
    apt-get update
    apt-get install -y \
      lsb-release \
      vim \
      software-properties-common \
      build-essential \
      tzdata \
      git \
      apt-transport-https \
      wget
  fi

  # Enable universe repository
  # apt-add-repository universe

  if ! which cmake
  then
    # Install CMake from kitware apt repo for newest possible version
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
    apt-get update
    apt-get install -y cmake
  fi

  # Install NVIDIA CUDA Toolkit
  if [ ! -e /usr/local/cuda-11.4/bin/cuda-gdb ]
  then
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
    wget https://developer.download.nvidia.com/compute/cuda/11.4.2/local_installers/cuda-repo-ubuntu2004-11-4-local_11.4.2-470.57.02-1_amd64.deb
    dpkg -i cuda-repo-ubuntu2004-11-4-local_11.4.2-470.57.02-1_amd64.deb
    apt-key add /var/cuda-repo-ubuntu2004-11-4-local/7fa2af80.pub
    apt-get update
    apt-get install -y \
      cuda
  fi

  export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}

  # Download open3d
  if [ ! -d /opt/source_code/Open3D ]
  then
    mkdir -p /opt/source_code
    cd /opt/source_code
    git clone --shallow-submodules --depth 1 --recursive https://github.com/intel-isl/Open3D -b v0.13.0
  fi

  # Install dependencies for building Open3D
  if [ ! -f /opt/source_code/installed_open3d_deps ]
  then
    cd /opt/source_code/Open3D
    export SUDO=command
    ./util/install_deps_ubuntu.sh assume-yes
    # avoid PYTHON_IN_PATH-NOTFOUND
    apt install -y \
      python-is-python3
    touch /opt/source_code/installed_open3d_deps
  fi

  # Build and install Open3D with Cuda support
  if [ ! -d /usr/local/bin/Open3D ]
  then
    cd /opt/source_code/Open3D
    # mkdir build
    cd build
    # cmake -DBUILD_CUDA_MODULE=ON -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
    make -j$(nproc)
    make install
  fi

  # Download PCL
  if [ ! -d /opt/source_code/pcl ]
  then
    mkdir -p /opt/source_code
    cd /opt/source_code
    git clone --shallow-submodules --depth 1 --recursive https://github.com/PointCloudLibrary/pcl.git -b pcl-1.12.0
  fi

  # Install dependencies for building PCL
  if [ ! -f /opt/source_code/installed_pcl_deps ]
  then
    apt update
    apt install -y \
      libboost-date-time-dev \
      libboost-filesystem-dev \
      libboost-iostreams-dev \
      libeigen3-dev \
      libflann-dev \
      libglew-dev \
      libgtest-dev \
      libopenni-dev \
      libopenni2-dev \
      libproj-dev \
      libqhull-dev \
      libqt5opengl5-dev \
      libusb-1.0-0-dev \
      libvtk7-dev \
      libvtk7-qt-dev \
      qtbase5-dev
      touch /opt/source_code/installed_pcl_deps
  fi

  # Build PCL with cuda support
  if [ ! -d /usr/local/share/pcl-1.12 ]
  then
    cd /opt/source_code/pcl
    mkdir build
    cd build
    cmake \
      -DBUILD_TESTS=OFF \
      -DWITH_CUDA=TRUE \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DCMAKE_BUILD_TYPE=Release \
      ..
    make -j$(nproc)
    make install
  fi

  # Download cupoch
  if [ ! -d /opt/source_code/cupoch ]
  then
    mkdir -p /opt/source_code
    cd /opt/source_code
    git clone --shallow-submodules --recursive https://github.com/neka-nat/cupoch.git
    cd cupoch
    git checkout a6a616ff4708b4cd5eee0c69012f90a6be6de1ff
  fi

  # Build and install cupoch
  if [ ! -d /usr/local/include/cupoch ]
  then
    cd /opt/source_code/cupoch
    mkdir build
    cd build
    cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      ..
    make -j$(nproc)
    make install
  fi

  rm -rf /var/lib/apt/lists/*
  apt-get clean


%environment
  export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}
  # export LANG=en_US.UTF-8
  # export ROS_PYTHON_VERSION=3
