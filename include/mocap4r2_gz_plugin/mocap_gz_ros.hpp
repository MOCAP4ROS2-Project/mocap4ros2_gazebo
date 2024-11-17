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

#ifndef GZ_PLUGINS__MOCAP_GZ_ROS_HPP_
#define GZ_PLUGINS__MOCAP_GZ_ROS_HPP_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"

#include "mocap_interfaces/msg/marker_array.hpp"
#include "mocap_interfaces/msg/rigid_body_array.hpp"
#include "mocap_interfaces/srv/create_rigid_body.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_control/ControlledLifecycleNode.hpp"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
class MocapGzRosPrivate;

class MocapGzRos
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  MocapGzRos();
  ~MocapGzRos() override = default;
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PostUpdate(
    const gz::sim::v8::UpdateInfo & _info,
    const gz::sim::v8::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<MocapGzRosPrivate> impl_;
};
} // namespace systems
} // inline namespace GZ_SIM_VERSION_NAMESPACE
} // namespace sim
} // namespace gz

#endif  // GZ_PLUGINS__MOCAP_GZ_ROS_HPP_
