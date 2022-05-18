# We don't need ubuntu:focal as it's already a layer in ros.
FROM ros:noetic-ros-core-focal

LABEL maintainer="Rico Jia"


# As of the creation of this file, the gpg key of ros docker for apt has expired. Therefore, we cannot run apt update and install at all without providing the key to apt-key
COPY gpg_key gpg_key 
RUN cat gpg_key | apt-key add -

RUN apt update && \
    apt install -yq \
    && apt-get install -yq --no-install-recommends \
      build-essential \
      unzip \
      curl \
      git \
      dbus \
      doxygen \
      doxygen-latex \
      gcc-arm-none-eabi \
      gdb-multiarch \
      libarmadillo-dev \
      liblapack-dev \
      libopenblas-dev \
      libnewlib-arm-none-eabi \
      libv4l-dev \
      libstdc++-arm-none-eabi-newlib \
      minicom \
      openocd \
      libxcb-xinerama0 \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-noetic-camera-info-manager \
      ros-noetic-tf-conversions \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-gazebo-ros-control \
      ros-noetic-catch-ros \
      ros-noetic-diagnostic-updater \
      ros-noetic-joint-state-controller \
      ros-noetic-joint-state-publisher \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      ros-noetic-robot-state-publisher \
      ros-noetic-rqt \
      ros-noetic-rqt-common-plugins \
      ros-noetic-rqt-console \
      ros-noetic-rviz \
      ros-noetic-self-test \
      ros-noetic-usb-cam \
      ros-noetic-xacro \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-moveit \
    ros-noetic-moveit-resources-prbt-moveit-config \
    ros-noetic-pilz-industrial-motion-planner \
    ros-noetic-rosserial \ 
      v4l-utils \
      vim \
#     # apt clean \
    ; exit 0


# #install joint_state_publisher_gui
RUN apt update \
    && apt upgrade -y \
    && apt install ros-noetic-joint-state-publisher-gui ; exit 0

RUN apt-get update && \
    apt-get install -yq ros-noetic-ddynamic-reconfigure \
    ros-noetic-librealsense2 \
    ros-noetic-realsense2-camera; exit 0

# Source the system's setup.bash
# need /bin/bash to make the container long running
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && /bin/bash"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN pip install opencv-contrib-python pyrealsense2

################################################################

# YOLOv5 🚀 by Ultralytics, GPL-3.0 license

# Start FROM Nvidia PyTorch image https://ngc.nvidia.com/catalog/containers/nvidia:pytorch
# FROM nvcr.io/nvidia/pytorch:21.10-py3

# Install linux packages
RUN apt update && apt install -y zip htop screen libgl1-mesa-glx

# Install python dependencies
COPY requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN pip uninstall -y torch torchvision torchtext
RUN pip install --no-cache -r requirements.txt albumentations wandb gsutil notebook \
    torch==1.10.2+cu113 torchvision==0.11.3+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
RUN pip install --no-cache -U torch torchvision

# Downloads to user config dir
# ADD https://ultralytics.com/assets/Arial.ttf /root/.config/Ultralytics/
################################################################

