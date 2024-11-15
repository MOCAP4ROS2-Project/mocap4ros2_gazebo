// Copyright 2024 Intelligent Robotics Lab
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

#include "gz_mocap4r2_plugin/gz_ros_mocap.hpp"
#include "mocap4r2_control/ControlledLifecycleNode.hpp"

using namespace gz;
using namespace sim;
using namespace systems;

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class gz::sim::systems::GzRosMocapPrivate : public mocap4r2_control::ControlledLifecycleNode
{
public:
  GzRosMocapPrivate();
  ~GzRosMocapPrivate() = default;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  void control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;

  void handleCreateRigidBody(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<mocap_interfaces::srv::CreateRigidBody::Request> request,
    const std::shared_ptr<mocap_interfaces::srv::CreateRigidBody::Response> response);

  std::vector<Entity> rigid_links_;
  std::vector<Entity> marker_links_;
  std::vector<std::string> rigid_links_names_;
  std::vector<std::string> marker_links_names_;
  std::map<int, mocap_interfaces::msg::Marker> markers_;
  std::map<std::string, std::vector<int>> rigid_body_markers_;
  std::map<std::string, std::string> rigid_body_orientation;
  Model model{kNullEntity};

  rclcpp_lifecycle::LifecyclePublisher<mocap_interfaces::msg::MarkerArray>::SharedPtr
    mocap_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap_interfaces::msg::RigidBodyArray>::SharedPtr
    mocap_rigid_body_pub_;
  rclcpp::Service<mocap_interfaces::srv::CreateRigidBody>::SharedPtr
    mocap_create_rigid_body_service_;
  int seq_{0};
};

GzRosMocapPrivate::GzRosMocapPrivate()
: ControlledLifecycleNode("gz_control")
{

  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

CallbackReturnT
GzRosMocapPrivate::on_configure(const rclcpp_lifecycle::State & state)
{
  mocap_markers_pub_ = create_publisher<mocap_interfaces::msg::MarkerArray>(
    "markers", rclcpp::QoS(1000));
  mocap_rigid_body_pub_ = create_publisher<mocap_interfaces::msg::RigidBodyArray>(
    "rigid_bodies", rclcpp::QoS(1000));
  mocap_create_rigid_body_service_ = create_service<mocap_interfaces::srv::CreateRigidBody>(
    "create_rigid_body",
    std::bind(
      &GzRosMocapPrivate::handleCreateRigidBody, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
GzRosMocapPrivate::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  mocap_markers_pub_->on_activate();
  mocap_rigid_body_pub_->on_activate();

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
GzRosMocapPrivate::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  mocap_markers_pub_->on_deactivate();
  mocap_rigid_body_pub_->on_deactivate();

  return ControlledLifecycleNode::on_deactivate(state);
}

void
GzRosMocapPrivate::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Starting mocap gazebo");
}

void
GzRosMocapPrivate::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Stopping mocap gazebo");
}

void GzRosMocapPrivate::handleCreateRigidBody(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<mocap_interfaces::srv::CreateRigidBody::Request> request,
  const std::shared_ptr<mocap_interfaces::srv::CreateRigidBody::Response> response)
{
  (void)request_header;

  RCLCPP_INFO(get_logger(), "Creating rigid body [%s]", request->rigid_body_name.c_str());
  rigid_body_markers_[request->rigid_body_name] = request->markers;
  rigid_body_orientation[request->rigid_body_name] = request->link_parent;

  response->success = true;
}

GzRosMocap::GzRosMocap()
{
  rclcpp::init(0, nullptr);
  impl_ = std::make_unique<GzRosMocapPrivate>();
}

void GzRosMocap::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  impl_->model = Model(_entity);

  auto sdf_element = _sdf->FindElement("rigid_link");

  while (sdf_element) {
    if (!sdf_element->Get<std::string>().empty()) {
      impl_->rigid_links_names_.push_back(sdf_element->Get<std::string>());
    } else {
      gzerr << "Empty rigid_link name" << std::endl;
    }

    sdf_element = sdf_element->GetNextElement("rigid_link");
  }

  sdf_element = _sdf->FindElement("marker_link");

  while (sdf_element) {
    if (!sdf_element->Get<std::string>().empty()) {
      impl_->marker_links_names_.push_back(sdf_element->Get<std::string>());
    } else {
      gzerr << "Empty marker_link name" << std::endl;
    }

    sdf_element = sdf_element->GetNextElement("marker_link");
  }

  for (const auto & link_name : impl_->rigid_links_names_) {
    auto link = impl_->model.LinkByName(_ecm, link_name);
    impl_->rigid_links_.push_back(link);
  }

  for (const auto & link_name : impl_->marker_links_names_) {
    auto link = impl_->model.LinkByName(_ecm, link_name);
    impl_->marker_links_.push_back(link);
  }
}

void GzRosMocap::PostUpdate(
  const gz::sim::v8::UpdateInfo & _info,
  const gz::sim::v8::EntityComponentManager & _ecm)
{
  rclcpp::spin_some(impl_->get_node_base_interface());

  if (impl_->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  mocap_interfaces::msg::MarkerArray ms;
  ms.header.stamp = impl_->now();
  ms.seq = impl_->seq_;
  ms.header.frame_id = "map";

  mocap_interfaces::msg::RigidBodyArray rbs;
  rbs.header.stamp = impl_->now();
  rbs.seq = impl_->seq_;
  rbs.header.frame_id = "map";

  impl_->seq_;
  int index = 1;

  for (int i = 0; i < impl_->rigid_links_.size(); i++) {
    auto link = impl_->rigid_links_[i];
    math::Pose3d pose = worldPose(link, _ecm);

    auto & pos = pose.Pos();
    auto & rot = pose.Rot();

    mocap_interfaces::msg::Marker m1;
    m1.header.stamp = impl_->now();
    m1.id_type = mocap_interfaces::msg::Marker::USE_INDEX;
    m1.marker_index = index++;
    m1.translation.x = pos.X();
    m1.translation.y = pos.Y();
    m1.translation.z = pos.Z() + 0.05;

    mocap_interfaces::msg::Marker m2;
    m2.header.stamp = impl_->now();
    m2.id_type = mocap_interfaces::msg::Marker::USE_INDEX;
    m2.marker_index = index++;
    m2.translation.x = pos.X() + 0.02;
    m2.translation.y = pos.Y();
    m2.translation.z = pos.Z() + 0.03;

    mocap_interfaces::msg::Marker m3;
    m3.header.stamp = impl_->now();
    m3.id_type = mocap_interfaces::msg::Marker::USE_INDEX;
    m3.marker_index = index++;
    m3.translation.x = pos.X();
    m3.translation.y = pos.Y() + 0.015;
    m3.translation.z = pos.Z() + 0.03;

    mocap_interfaces::msg::RigidBody rb;
    rb.header.stamp = impl_->now();
    rb.rigid_body_name = impl_->rigid_links_names_[i];
    rb.pose.position.x = pos.X();
    rb.pose.position.y = pos.Y();
    rb.pose.position.z = pos.Z();
    rb.pose.orientation.x = rot.X();
    rb.pose.orientation.y = rot.Y();
    rb.pose.orientation.z = rot.Z();
    rb.pose.orientation.w = rot.W();

    rb.markers = {m1, m2, m3};

    ms.markers.push_back(m1);
    ms.markers.push_back(m2);
    ms.markers.push_back(m3);
    rbs.rigid_bodies.push_back(rb);

    impl_->markers_[m1.marker_index] = m1;
    impl_->markers_[m2.marker_index] = m2;
    impl_->markers_[m3.marker_index] = m3;
  }

  for (const auto & link : impl_->marker_links_) {
    math::Pose3d pose = worldPose(link, _ecm);

    auto & pos = pose.Pos();

    mocap_interfaces::msg::Marker m1;
    m1.header.stamp = impl_->now();
    m1.id_type = mocap_interfaces::msg::Marker::USE_INDEX;
    m1.marker_index = index++;
    m1.translation.x = pos.X();
    m1.translation.y = pos.Y();
    m1.translation.z = pos.Z();

    ms.markers.push_back(m1);
    impl_->markers_[m1.marker_index] = m1;
  }

  for (const auto & [rigid_body_name, markers] : impl_->rigid_body_markers_) {
    mocap_interfaces::msg::RigidBody rb;
    geometry_msgs::msg::Point rigid_body_pose;
    rb.header.stamp = impl_->now();
    rb.rigid_body_name = rigid_body_name;

    for (const auto & marker_index : markers) {
      auto it = impl_->markers_.find(marker_index);
      if (it != impl_->markers_.end()) {
        rb.markers.push_back(it->second);
        rigid_body_pose.x += it->second.translation.x;
        rigid_body_pose.y += it->second.translation.y;
        rigid_body_pose.z += it->second.translation.z;
      }
    }

    geometry_msgs::msg::Point centroid;
    centroid.x = rigid_body_pose.x / rb.markers.size();
    centroid.y = rigid_body_pose.y / rb.markers.size();
    centroid.z = rigid_body_pose.z / rb.markers.size();

    rb.pose.position = centroid;

    if (impl_->rigid_body_orientation.find(rigid_body_name) !=
      impl_->rigid_body_orientation.end())
    {
      auto link = impl_->model.LinkByName(_ecm, impl_->rigid_body_orientation[rigid_body_name]);
      math::Pose3d pose = worldPose(link, _ecm);
      auto & rot = pose.Rot();
      rb.pose.orientation.x = rot.X();
      rb.pose.orientation.y = rot.Y();
      rb.pose.orientation.z = rot.Z();
      rb.pose.orientation.w = rot.W();
    }

    rbs.rigid_bodies.push_back(rb);
  }

  if (impl_->mocap_markers_pub_->get_subscription_count() > 0) {
    impl_->mocap_markers_pub_->publish(ms);
  }

  if (impl_->mocap_rigid_body_pub_->get_subscription_count() > 0) {
    impl_->mocap_rigid_body_pub_->publish(rbs);
  }
}

GZ_ADD_PLUGIN(GzRosMocap, gz::sim::System,
  GzRosMocap::ISystemConfigure,
  GzRosMocap::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(GzRosMocap, "gz::sim::systems::GzRosMocap")
