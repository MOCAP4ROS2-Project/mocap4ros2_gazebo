// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <memory>
#include <string>
#include <utility>

#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_mocap.hpp>
#include <gazebo_ros/node.hpp>

#include "mocap_msgs/msg/markers.hpp"
#include "mocap_msgs/msg/rigid_body.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "mocap_control/ControlledLifecycleNode.hpp"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMocap)

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/// Class to hold private data members (PIMPL pattern)
class GazeboRosMocapPrivate : public mocap_control::ControlledLifecycleNode
{
public:
  GazeboRosMocapPrivate();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  void control_start(const mocap_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg) override;

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  std::string link_name_;

  /// ROS communication.
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::Markers>::SharedPtr mocap_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_msgs::msg::RigidBody>::SharedPtr mocap_rigid_body_pub_;
  int frame_number_{0};
};

GazeboRosMocapPrivate::GazeboRosMocapPrivate()
: ControlledLifecycleNode("gazebo_control")
{
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}


CallbackReturnT
GazeboRosMocapPrivate::on_configure(const rclcpp_lifecycle::State & state)
{
  mocap_markers_pub_ = create_publisher<mocap_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap_rigid_body_pub_ = create_publisher<mocap_msgs::msg::RigidBody>(
    "rigid_bodies", rclcpp::QoS(1000));

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
GazeboRosMocapPrivate::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  mocap_markers_pub_->on_activate();
  mocap_rigid_body_pub_->on_activate();

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
GazeboRosMocapPrivate::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  mocap_markers_pub_->on_deactivate();
  mocap_rigid_body_pub_->on_deactivate();

  return ControlledLifecycleNode::on_deactivate(state);
}

void
GazeboRosMocapPrivate::control_start(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Starting mocap gazebo");
}

void
GazeboRosMocapPrivate::control_stop(const mocap_control_msgs::msg::Control::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Stopping mocap gazebo");
}

GazeboRosMocap::GazeboRosMocap()
: impl_(std::make_unique<GazeboRosMocapPrivate>())
{
}

GazeboRosMocap::~GazeboRosMocap()
{
  impl_->update_connection_.reset();
}

void GazeboRosMocap::Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMocap::OnUpdate, this));

  auto logger = impl_->get_logger();

  if (!sdf->HasElement("link_name")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <link_name> wasn't set,"
      "therefore it's been set as 'base_mocap'.");
    impl_->link_name_ = "base_mocap";
  } else {
    impl_->link_name_ = sdf->GetElement("link_name")->Get<std::string>();
  }
  impl_->world_ = _parent->GetWorld();
  impl_->link_ = boost::dynamic_pointer_cast<physics::Link>(
    impl_->world_->EntityByName(impl_->link_name_));

  RCLCPP_INFO(logger, "Plugin MOCAP loaded for [%s] body", impl_->link_name_.c_str());
}

void GazeboRosMocap::OnUpdate()
{
  rclcpp::spin_some(impl_->get_node_base_interface());

  if (impl_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  ignition::math::v6::Pose3d pose = impl_->link_->WorldPose();

  auto & pos = pose.Pos();
  auto & rot = pose.Rot();

  if (impl_->mocap_markers_pub_->get_subscription_count() > 0) {
    mocap_msgs::msg::Marker m1;
    m1.marker_index = 1;
    m1.translation.x = pos.X();
    m1.translation.y = pos.Y();
    m1.translation.z = pos.Z() + 0.05;

    mocap_msgs::msg::Marker m2;
    m2.marker_index = 2;
    m2.translation.x = pos.X() + 0.02;
    m2.translation.y = pos.Y();
    m2.translation.z = pos.Z() + 0.03;

    mocap_msgs::msg::Marker m3;
    m3.marker_index = 3;
    m3.translation.x = pos.X();
    m3.translation.y = pos.Y() + 0.015;
    m3.translation.z = pos.Z() + 0.03;

    mocap_msgs::msg::Markers ms;
    ms.header.stamp = impl_->now();
    ms.header.frame_id = "mocap";
    ms.frame_number = impl_->frame_number_++;
    ms.markers = {m1, m2, m3};

    impl_->mocap_markers_pub_->publish(ms);
  }

  if (impl_->mocap_rigid_body_pub_->get_subscription_count() > 0) {
    mocap_msgs::msg::RigidBody rb;
    //rb.index = 1;
    rb.header.stamp = impl_->now();
    rb.header.frame_id = "mocap";
    rb.frame_number = impl_->frame_number_++;
    rb.pose.position.x = pos.X();
    rb.pose.position.y = pos.Y();
    rb.pose.position.z = pos.Z();
    rb.pose.orientation.x = rot.X();
    rb.pose.orientation.y = rot.Y();
    rb.pose.orientation.z = rot.Z();
    rb.pose.orientation.w = rot.W();

    impl_->mocap_rigid_body_pub_->publish(rb);
  }
}

}  // namespace gazebo
