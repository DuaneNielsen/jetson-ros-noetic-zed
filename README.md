# Dockerfile for ROS1 Noetic ZED node with custom yolov7 network

## to build 

```
dockerbuild () {
  local command="sudo docker build ."
  local version="--build-arg L4T_MAJOR_VERSION=35
                 --build-arg L4T_MINOR_VERSION=1   
         --build-arg L4T_PATCH_VERSION=0   
         --build-arg ZED_SDK_MAJOR=3   
         --build-arg ZED_SDK_MINOR=8   
         --build-arg JETPACK_MAJOR=5   
         --build-arg JETPACK_MINOR=0.2   
         --build-arg L4T_BASE_IMAGE=l4t-jetpack"
  $command $version "$@"
}
dockerbuild -t noetic-ros-roszed-l4t-r35.1.0
```

## start the ZED node
```
sudo docker run noetic-ros-roszed-l4t-r35.1.0 --runtime nvidia --network host --gpus all --privileged
```

## to run in interactive mode

```
# override the entrypoint of the container
dockerrun () {
  local command="sudo docker run"
  local options="--runtime nvidia
                 -it
                 --rm
                 --network host
                 -e ROS_LOG_LEVEL=debug
                 --gpus all
                 --privileged
                 --entrypoint /bin/bash"
  local image=$1
  $command $options $image
}
dockerrun noetic-ros-roszed-l4t-r35.1.0
```

### ros commands

start ros zed

```
. ~/zed_catkin_ws/devel/setup.bash
roslaunch zed_wrapper zed2i.launch
```

start ros zed with RVIZ monitoring
```
. ~/zed_catkin_ws/devel/setup.bash
roslaunch zed_display_rviz display_zed2i.launch
```

