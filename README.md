# MOCAP4ROS2 for Gazebo

[![CI](https://github.com/MOCAP4ROS2-Project/mocap4ros2_gazebo/actions/workflows/ci.yaml/badge.svg?branch=rolling)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_gazebo/actions/workflows/ci.yaml)

* `mocap4r2_gz_plugin`: a simulated mocap room for Gazebo (Jazzy and newer). It streams
  markers, rigid bodies and skeletons of the links you choose as a MOCAP4ROS2 system, or
  plays OptiTrack Motive so that the real OptiTrack driver connects to Gazebo.
* `mocap4r2_gz_demos`: robot ground truth and human motion demos.

```bash
ros2 launch mocap4r2_gz_demos robot_groundtruth.launch.py system:=optitrack
ros2 launch mocap4r2_gz_demos human_motion.launch.py
```

Documentation: https://mocap4ros2-project.github.io/latest/systems/gazebo.html

## Acknowledgements

MOCAP4ROS2 started as a Focused Technical Project of [ROSIN](https://www.rosin-project.eu/),
which received funding from the European Union’s Horizon 2020 research and innovation
programme under grant agreement no. 732287. Developed at the
[Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/), Rey Juan Carlos University.
