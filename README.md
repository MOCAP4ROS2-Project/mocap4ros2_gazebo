
# gazebo_mocap_plugin

This package provides you with a gazebo plugin that allows you to simulate the use of Motion Capture Markers on your simulated robot.

## How to use

Add in your robot model the following, as done in [This sample model](https://github.com/MOCAP4ROS2-Project/mocap4ros2_gazebo/blob/main/models/waffle.model):

```
      <link name="base_mocap">
        <pose>-0.052 0 0.141 0 0 0</pose>
        <inertial>
          <pose>-0.052 0 0.141 0 0 0</pose>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0.000</ixy>
            <ixz>0.000</ixz>
            <iyy>0.001</iyy>
            <iyz>0.000</iyz>
            <izz>0.001</izz>
          </inertia>
          <mass>0.125</mass>
        </inertial>

        <collision name="mocap_sensor_collision">
          <pose>-0.052 0 0.160 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.0508</radius>
              <length>0.055</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="mocap_sensor_visual">
          <pose>-0.064 0 0.141 0 0 0</pose>
          <geometry>
            <mesh>
              <uri>model://gazebo_mocap_plugin/meshes/rigid_body.dae</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
      
      ...

      <plugin name="gazebo_ros_mocap" filename="libgazebo_ros_mocap.so">
        <link_name>base_mocap</link_name>
      </plugin>
```

## Run the sample

To see it in action, just type:

```
ros2 launch gazebo_mocap_plugin tb3_simulation_launch.py
```

Open gzclient in other terminal:

```
gzclient
```

And check that topics `/markers` and `/rigid_bodies` are availables


Example of full integration with `mocap_control`.

[![](https://img.youtube.com/vi/i9U_T0Ti6Oo/0.jpg)](https://www.youtube.com/watch?v=i9U_T0Ti6Oo&feature=youtu.be "Click to play on You Tube")


## Markers and rigid body
<img src="https://user-images.githubusercontent.com/3810011/178335627-080f8d5a-7b6f-40d2-8038-caf73e7cf8d9.png" width="500">
<img src="https://user-images.githubusercontent.com/3810011/178335622-02e126f7-ec96-41a0-9936-38af589c5a2d.png" width="500">
