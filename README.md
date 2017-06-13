# The Picky Robot

This demo uses a UR5 robot and a depth camera to detect objects
and push them off the table. It is a vision-based manipulation demo
that does not require a gripper.

This demo requires several dependencies:
  - python-urx: a ROS 2 fork (see the `ros2` branch) is available at
    https://github.com/Kukanani/python-urx.
  - A suitable depth camera library: this demo uses defaults that work
    with the Orbbec Astra, and the included intra-process launcher file
    depends on and uses the Astra driver. It should be possible to switch
    out to a different depth camera by adding dependencies, changing camera
    parameters, and writing a new intra-process launcher to replace
    `linemod_pipeline.yaml` (which is a short file). A ROS 2 fork of the astra
    camera driver is available at https://github.com/ros2/ros_astra_camera.
  - The URX driver also requires some python dependencies - `numpy`, `yaml`, and
    `math3d`. `math3d`.
  - The LINEMOD detector used in this pipeline requires extra OpenCV 3 modules
    that are only available in the `opencv_contrib` package. To get access
    to this functionality, you must install OpenCV3 from source with the `rgbd`
    module enabled. For details on how to install OpenCV 3 from source, please
    see please see the readme at https://github.com/opencv/opencv_contrib.