ARG VERSION=latest
ARG REGISTRY
FROM ${REGISTRY}uobflightlabstarling/starling-controller-base:latest

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        python-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros_ws/src && cd /ros_ws/src && git clone https://github.com/mhl787156/starling_simple_offboard.git

COPY starling_allocator /ros_ws/src/starling_allocator
COPY starling_allocator_msgs /ros_ws/src/starling_allocator_msgs

# Build the package
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH \
    && cd /ros_ws \
    && colcon build --packages-select starling_allocator starling_allocator_msgs simple_offboard_msgs\
    && rm -r build

CMD ["ros2", "run", "starling_allocator", "allocator"]
