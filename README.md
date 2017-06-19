# The Picky Robot

This demo uses a UR5 robot and a depth camera to detect objects
and push them off the table. It is a vision-based manipulation demo
that does not require a gripper.

## Dependencies

This demo requires several dependencies:
  - python-urx: a ROS 2 fork (see the `ros2` branch) is available at
    https://github.com/Kukanani/python-urx. This should be installed
    automatically
  - A suitable depth camera library: this demo uses defaults that work
    with the Orbbec Astra, and the included intra-process launcher file
    depends on and uses the Astra driver. It should be possible to switch
    out to a different depth camera by adding dependencies, changing camera
    parameters, and writing a new intra-process launcher to replace
    `linemod_pipeline.yaml` (which is a short file). A ROS 2 fork of the astra
    camera driver is available at https://github.com/ros2/ros_astra_camera.

These first two dependencies should be installed automatically if you follow the
installation procedure below. However, there are some more dependencies which
must be installed manually:

  - The URX driver also requires some python dependencies - `numpy`, `yaml`, and
    `math3d`. `math3d`.
  - The LINEMOD detector used in this pipeline requires extra OpenCV 3 modules
    that are only available in the `opencv_contrib` package. To get access
    to this functionality, you must install OpenCV3 from source with the `rgbd`
    module enabled. For details on how to install OpenCV 3 from source, please
    see please see the readme at https://github.com/opencv/opencv_contrib.

## Installation (Ubuntu 16.04)

Install dependencies (above). Also install the following:
```
sudo apt install python3-vcstool
```

Download the repos file and use it to gather sources. From the root of your ROS2
workspace, run:
```
wget https://raw.githubusercontent.com/Kukanani/picky_robot/ros2/picky_robot.repos
vcs import src < ros2.repos
```

Then, build your workspace.

## Running

  - You will need to acquire training data from the `linemod_basic_detector`,
    please see that package for more details. The template YAML files must point
    at valid mesh paths for the detection pipeline to work, since it does
    virtual renders of objects on-the-fly. We used absolute paths, which is not
    reusable, but ensures that the pipeline can be run from any directory.
    Relative path support is a major area where the demo could be improved.
  - In one terminal, run `ur5_pusher.py`. This connects to and controls the
    robot.
  - In a second terminal, run `linemod_pipeline <your_template_file> b`.
    Detection windows should appear, showing a camera feed with detected
    templates superimposed.
  - In a third terminal, run `picky_robot.py`. This filters the detection
    results and signals the pusher to push when and where is appropriate.