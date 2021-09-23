# Starling Allocator

This project provides a method of automatically allocating a number of trajectories to the (mavros-enabled) vehicles visible on the ros network. This was designed for use in the Bristol Flight Arena as part of the [ProjectStarling](https://github.com/UoBFlightLab/ProjectStarling) project.

This project was developed in conjunction with [the GUI](https://github.com/mhl787156/starling_ui_dashly) and a [simple offboard mavros px4 controller](https://github.com/mhl787156/starling_simple_offboard), although it can be used on its as long as the interfaces are satisfied:

* Input `AllocateTrajectories.srv`
* Output `SubmitTrajectory.srv`

There are currently 3 supported methods of allocation:

1. **manual** where the caller supplies a list of `manual_allocation_targets`
2. **nearest** where the allocator polls for the current list of vehicles (topics of format `/<vehicle>/mavros/...`) and maps vehicles to trajectories with the nearest starting point
3. **random** where the trajectory list is shuffled to produce a random allocation.

## Building and Running

### Running Natively

First clone this repository into your ros2 workspace, and also clone the dependent repository
```
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/mhl787156/starling_allocator.git
git clone https://github.com/mhl787156/starling_simple_offboard.git # This is for the simple_offboard messages
```

Then in the root of the workspace (i.e. ros_ws) you can build the project using
```
source /opt/ros/foxy/setup.bash
colcon build --packages-select starling_allocator starling_allocator_msgs simple_offboard_msgs
```

Then to run the project
```
ros2 run starling_allocator allocator
```

### Running via Docker

This project is also available as a prebuilt docker image available as part of ProjectStarling. In order to run the project run the following (ensure docker is already installed):

```
docker pull uobflightlabstarling/starling-allocator
docker run --it -rm uobflightlabstarling/starling-allocator
```

> **NOTE:** If you are running this with the ProjectStarling `docker-compose`, do not forget to add `--network projectstarling_default` to the `docker run` command.


## Interface

The key input service is on `\submit_trajectories` which has the following interface:

```
# List of trajectories to be allocated to various drones
trajectory_msgs/JointTrajectory[] trajectories

# List of trajectory types (i.e. position or attitude)
string[] trajectory_types

# Manual Allocation vehicle names in order
string[] manual_allocation_targets

# Allocation method (e.g. manual or nearest or random)
string method

# Interpolation method (e.g. linear or cubic or other, defaults to cubic)
string interpolation_method

# Automatically arm and takeoff (default true)
bool auto_arm

# Automatically takeoff (default true)
bool auto_takeoff
---

bool success
string message
Allocation[] allocation
```

There must be at least 1 trajectory in the trajectories list with a matching trajectory type, otherwise an error will be returned. The `interpolation_method`, `auto_arm` and `auto_takeoff` fields are directly passed to the controller.