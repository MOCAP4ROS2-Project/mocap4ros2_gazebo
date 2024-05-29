
# gazebo_mocap4r2_plugin

This package provides you with a gazebo plugin that allows you to simulate the use of Motion Capture Markers on your simulated robot.

## How to use

Add in your robot model the following, as done in [This sample model](https://github.com/MOCAP4ROS2-Project/mocap4ros2_gazebo/blob/main/models/waffle.model):

```xml
    <link name="base_mocap">
        <pose>"0 0 0.577 0 0 0</pose>
        <inertial>
          <mass value="0.125"/>
          <origin xyz="0 0 0.577"/>
          <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
        </inertial>

        <collision name="mocap_sensor_collision">
          <pose>0 0 0.577 0 0 0</pose>
          <geometry>
            <cylinder length="0.055" radius="0.0508"/>
          </geometry>
        </collision>

        <visual name="mocap_sensor_visual">
          <origin rpy="0 0 0" xyz=" 0 0 0"/>
          <geometry>
            <mesh filename="package://gazebo_mocap4r2_plugin/meshes/rigid_body.dae" scale="0.001 0.001 0.001"/>
          </geometry>
        </visual>
      </link>
    <gazebo>
    <plugin name="gazebo_ros_mocap" filename="libgazebo_ros_mocap.so">
      <model_name>robot</model_name>
      <rigid_link>base_footprint</rigid_link>
    </plugin>
    </gazebo>
```

If what you want is to add markers, you can do so by adding the following line in your plugin. For this case, we would add a marker in base_footprint:
```xml
<plugin name="gazebo_ros_mocap" filename="libgazebo_ros_mocap.so">
    <model_name>robot</model_name>
    <marker_link>base_link</marker_link>
</plugin>
 ```

And if you have 3 or more markers within your environment, you can create new rigid_bodies from them. To do this, you have to choose the orientation through a link and add the index of the markers you want to add.
```bash
ros2 service call /create_rigid_body mocap4r2_msgs/srv/CreateRigidBody 'rigid_body_name: '\'new_rigid''\''
link_parent: '\'base_link''\''
markers: [1, 2, 5, 8]'
```

## Run the sample

To see it in action, just type:

```bash
ros2 launch gazebo_mocap4r2_plugin tb3_simulation_launch.py
```

Open gzclient in other terminal:

```bash
gzclient
```

And check that topics `/markers` and `/rigid_bodies` are availables


Example of full integration with `mocap_control`.

[![](https://img.youtube.com/vi/i9U_T0Ti6Oo/0.jpg)](https://www.youtube.com/watch?v=i9U_T0Ti6Oo&feature=youtu.be "Click to play on You Tube")


## Markers and rigid body
<img src="https://user-images.githubusercontent.com/3810011/178335627-080f8d5a-7b6f-40d2-8038-caf73e7cf8d9.png" width="500">
<img src="https://user-images.githubusercontent.com/3810011/178335622-02e126f7-ec96-41a0-9936-38af589c5a2d.png" width="500">
