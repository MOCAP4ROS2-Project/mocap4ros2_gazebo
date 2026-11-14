^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_gz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Rewritten (Jazzy and newer, gz vendor packages): a world plugin simulating a mocap room; markers, rigid bodies and skeletons of the configured links, streamed as a MOCAP4ROS2 system or as OptiTrack Motive for the real OptiTrack driver.
* Simulation time stamps, no publishing while paused, one rclcpp::init for all plugins, installed with an environment hook (nothing in $HOME).
* GaitAnimator: a walking mannequin for human motion demos.
* Contributors: Francisco Martín Rico, Juan Carlos Manzanares Serrano, Alberto García, José Miguel Guerrero
