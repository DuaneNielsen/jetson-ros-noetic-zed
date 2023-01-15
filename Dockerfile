#
# this dockerfile roughly follows the 'Installing from source' from:
#   http://wiki.ros.org/noetic/Installation/Source
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.1.0
FROM ${BASE_IMAGE}

ARG ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

# Glow documentation
RUN apt-get update && \
    sudo mkdir -p /etc/apt/keyrings && \
    sudo apt-get install curl -y && \
    curl -fsSL https://repo.charm.sh/apt/gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/charm.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/charm.gpg] https://repo.charm.sh/apt/ * *" | sudo tee /etc/apt/sources.list.d/charm.list && \
    sudo apt update && sudo apt install glow

WORKDIR /workspace

#
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          libpython3-dev \
          python3-rosdep \
          python3-rosinstall-generator \
          python3-vcstool \
          build-essential && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*


#
# download/build the ROS source
#
RUN mkdir ros_catkin_ws && \
    cd ros_catkin_ws && \
    rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
    mkdir src && \
    vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
    apt-get update && \
    rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# install ZED
ARG L4T_MAJOR_VERSION
ARG L4T_MINOR_VERSION
ARG L4T_PATCH_VERSION
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update -y || true
RUN sudo apt install zstd -y
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https -y && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent runtime_only && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*

# ZED Python API
RUN apt-get update -y || true
RUN apt-get install --no-install-recommends python3 python3-pip python3-dev python3-setuptools build-essential -y && \
    wget download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py && \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install cython wheel && \
    python3 -m pip install numpy *.whl && \
    apt-get remove --purge build-essential python3-dev -y && apt-get autoremove -y && \
    rm *.whl ; rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so


# ZED node with custom yolov7
RUN mkdir /workspace/zed_catkin_ws && \
    mkdir /workspace/zed_catkin_ws/src
RUN sudo apt-get update && sudo apt-get -y install git-lfs
WORKDIR /workspace/zed_catkin_ws/src
ARG YOLO_VERSION=0.2
RUN git clone https://huggingface.co/Duane/zed-ros-yolo-weights
ADD https://api.github.com/repos/duanenielsen/zed-ros-wrapper/git/refs/heads/master zed_ros_wrapper_git_version.json
RUN git clone --recursive https://github.com/duanenielsen/zed-ros-wrapper.git
RUN apt-get update
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    cd /workspace/zed_catkin_ws/ && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release


#
# setup entrypoint
#
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
COPY README.md /root/README.md
RUN echo 'glow ~/README.md' >> /root/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /
