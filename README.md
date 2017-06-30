# The Picky Robot

This demo uses a UR5 robot and a depth camera to detect objects
and push them off the table. It is a vision-based manipulation demo
that does not require a gripper.

## Run via Docker

The easiest way to run the demo is using Docker. You should
[install docker](https://docs.docker.com/engine/installation/)
and [nvidia-docker](https://github.com/NVIDIA/nvidia-docker). You'll also have
to have an arm and a depth camera. The demo is currently designed to use a
Universal Robots UR5 and an Orbbec Astra. You may also have to [install the
Orbbec udev rules](http://wiki.ros.org/astra_camera) on your host machine.

The docker container runs in privileged mode to allow access to USB (camera) and
ethernet (robot).

To build the docker container, navigate to the `picky_robot_docker/docker`
directory and run the command

````
./build_picky_robot_docker.sh
```

This will take quite some time, since it must download dependencies, download
the data files for the detected objects, download ROS 2 source code, and build
everything. Once it's done, you can run

```
./run_picky_robot_docker.sh
```

And, once inside the container, run

```
cd picky_robot_data
python3 ../install_isolated/picky_robot/share/launch/ur5_launch.py --linemod_templates cupnoodles_penne.yml -r true -p true
```

The `-p true` and `-r true` flags enable manipulation of the two different object types.

### Building Manually

This method will give you the most control, but there are quite a few installation
steps.

## Dependencies

  - You should have a ROS 2 workspace: follow the [installation instructions](https://github.com/ros2/ros2/wiki/Installation).
  - A suitable depth camera library: this demo uses defaults that work
    with the Orbbec Astra, and the included intra-process launcher file
    depends on and uses the Astra driver. It should be possible to switch
    out to a different depth camera by adding dependencies, changing camera
    parameters, and writing a new intra-process launcher to replace
    `linemod_pipeline.yaml` (which is a short file). A ROS 2 fork of the astra
    camera driver is available at https://github.com/ros2/ros_astra_camera.
  - python-urx: Communicates with the robot arm. This should be installed
    automatically when following the build procedure below. A ROS 2 fork (see
    the `ros2` branch) is available at https://github.com/Kukanani/python-urx.
  - The last two items listed above should be installed automatically. There are
    also some additional dependencies which must be installed manually, like so:

```
# special depends for ORK renderer
apt-get install -y \
      libboost-dev \
      libassimp-dev \
      freeglut3-dev \
      libgl1-mesa-dev \
      libfreeimage-dev \
      libxmu-dev \
      libxi-dev \
      libsdl1.2-dev \
      libosmesa6-dev \
      libusb-1.0-0-dev \
      libudev-dev

# special depends for Astra camera driver
apt-get install -y \
  libboost-system-dev \
  libboost-thread-dev

# special depends for demo
pip3 install --upgrade pip
pip3 install numpy math3d yaml
```

### Installation (Ubuntu 16.04)

Install dependencies (above).

Download the repos file and use it to gather sources. From the root of your ROS2
workspace, run:
```
wget https://raw.githubusercontent.com/Kukanani/picky_robot/ros2/picky_robot.repos
vcs import src < ros2.repos
```

Then, build your workspace.

### Run the demo

#### Training

  - You will need to acquire training data from the `linemod_basic_detector`,
    please see that package for more details. The template YAML files must point
    at valid mesh paths for the detection pipeline to work, since it does
    virtual renders of objects on-the-fly. It's a good idea to store the mesh
    files next to the .yml file, in the same directory.

#### Testing

  - You should run from the same folder that contains your .yml and mesh files.
  - In a terminal, run `ur5_launch.py --linemod_templates <your_template_file>`.
    Detection windows should appear, showing a camera feed with detected
    templates superimposed. When an object's detection is stable for a few frames,
    the robot should push it off the table.
  - The transformation between the camera frame and the world frame can be changed
    by using a YAML file and passing it to the launch script as a command-line
    argument. Please see `ur5_launch.py --help` and
    `picky_robot/launch/osrf_calib.yaml` for more details.
