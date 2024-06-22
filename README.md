# Cafeteriabot Webapp

The final goal of the project is to set up an automatic system that allows for a trash table in cafeteria to be found and picked up by a robot called `cleaner` and taken to a back room, where it will be emptied.

![main](.assets/main.png)

**Note:** This repository contains Cafeteriabot web application code, for complete project check [here](https://github.com/llabhishekll/cafeteriabot_project).

![ros](https://img.shields.io/badge/ROS2-humble-red)![cpp](https://img.shields.io/badge/python-3.8+-blue)

## Structure

```text
.
├── launch
│   └── cafeteriabot_webapp.launch.py
├── scripts
│   ├── lib
│   │   ├── colladaloader.js
│   │   ├── colladaloader2.js
│   │   ├── easel.js
│   │   ├── eventemitter2.js
│   │   ├── mjpegcanvas.js
│   │   ├── ros2d.js
│   │   ├── ros3d.js
│   │   ├── roslib.js
│   │   ├── stlloader.js
│   │   └── three.js
│   └── app.js
├── styles
│   └── app.css
├── CMakeLists.txt
├── Dockerfile
├── README.md
├── docker-compose.yml
├── index.html
├── package.xml
└── ros_entrypoint.sh
```

## Setup

#### Distribution

Use docker for quick-start (for both ROS1 or ROS2):

```bash
# using docker for ROS1
$ docker run -ti --rm --name local-ros-noetic ros:noetic
```

```bash
# using docker for ROS2
$ docker run -ti --rm --name local-ros-humble ros:humble
```

#### Build (Package)

Now, create a catkin workspace, clone the package:

```bash
# setup directory
$ mkdir ~/ros2_ws/src/
$ git clone <repo_name> ~/ros2_ws/src/
```

Install the required packages (dependency) mentioned in `package.xml` using `apt`:

```bash
# check if package is available
$ ros2 pkg list
$ ros2 node list
```

```bash
# update path to installed packages
$ source /opt/ros/humble/setup.bash
```

To build locally or inside docker use the following commands:

```bash
# execute build
$ cd ~/ros2_ws && colcon build
$ source ~/ros2_ws/install/setup.bash
```

## Launch

![webapp](/Users/abhishekverma/Github/cafeteriabot_webapp/.assets/webapp.gif)

The `cafeteriabot_webapp.launch.py` launch file contains the followings nodes:

- `rosbridge_websocket_launch.xml` : WebSocket server for enabling communication between non-ROS programs and ROS
- `python http server` : A system process to start HTTP server using Python's built-in `http.server` module.

```bash
# launch complete webapp
$ ros2 launch cafeteriabot_webapp cafeteriabot_webapp.launch.py
```

The commands below are specific to platform, for general case this will be `localhost`.

```bash
# webpage and rosbridge address
$ webpage_address && rosbridge_address
```

#### Key Feature

- `Parameter Server` : Adjust key parameter of detection system on runtime.
- `Trash Table` : View midpoint, centroid and other values in real-time.
- `Robot Localization` : Provides real-time information about the robot's position and orientation on map.
- `Terminal` : Show system log value and other critical information.
- `Joystick` : Enables manual control of the robot's movement using a joystick interface.
- `Dock Server Status` : Monitors the status and availability of docking action server.
- `Velocity and Odometry` : Displays real-time velocity and odometry information of the robot.

## Tools

System tool/modules used for project development.

- `Applications` : [vs-code](https://code.visualstudio.com/), [ros-extensions](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros) and [docker-desktop](https://docs.docker.com/get-docker/).
- `ROS` : [ros-docker-images](https://hub.docker.com/_/ros/) (`humble`, `noetic`) or [build-source](https://www.ros.org/blog/getting-started/).

## License

Distributed under the MIT License. See `LICENSE.txt` for more information.
