// Copyright 2026 Intelligent Robotics Lab
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

// Gazebo model plugin: a walking pattern for an articulated mannequin (hips, knees,
// shoulders, elbows), moving along a circle. Kinematic: it sets the joint positions
// and the model pose every step, for mocap demos of human motion.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace mocap4r2_gz_plugin
{

class GaitAnimator
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    frequency_ = sdf->HasElement("frequency") ? sdf->Get<double>("frequency") : 0.9;
    radius_ = sdf->HasElement("radius") ? sdf->Get<double>("radius") : 1.5;
    speed_ = sdf->HasElement("speed") ? sdf->Get<double>("speed") : 0.6;
    height_ = sdf->HasElement("height") ? sdf->Get<double>("height") : 0.95;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double ph = 2.0 * M_PI * frequency_ * t;
    const double s = std::sin(ph);
    const std::map<std::string, double> angles = {
      {"hip_l", 0.45 * s}, {"hip_r", -0.45 * s},
      {"knee_l", -0.7 * std::max(0.0, std::sin(ph - 1.2))},
      {"knee_r", -0.7 * std::max(0.0, std::sin(ph + M_PI - 1.2))},
      {"shoulder_l", -0.35 * s}, {"shoulder_r", 0.35 * s},
      {"elbow_l", -0.35}, {"elbow_r", -0.35}};
    for (const auto & [name, angle] : angles) {
      auto j = model_.JointByName(ecm, name);
      if (j != gz::sim::kNullEntity) {
        gz::sim::Joint(j).ResetPosition(ecm, {angle});
      }
    }
    // Walk along a circle, facing forward, with a small vertical oscillation.
    const double th = speed_ / radius_ * t;
    const double z = height_ + 0.02 * std::cos(2.0 * ph);
    model_.SetWorldPoseCmd(
      ecm, gz::math::Pose3d(radius_ * std::cos(th), radius_ * std::sin(th), z, 0, 0, th + M_PI_2));
  }

private:
  gz::sim::Model model_;
  double frequency_{0.9};
  double radius_{1.5};
  double speed_{0.6};
  double height_{0.95};
};

}  // namespace mocap4r2_gz_plugin

GZ_ADD_PLUGIN(
  mocap4r2_gz_plugin::GaitAnimator, gz::sim::System,
  mocap4r2_gz_plugin::GaitAnimator::ISystemConfigure,
  mocap4r2_gz_plugin::GaitAnimator::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(mocap4r2_gz_plugin::GaitAnimator, "mocap4r2_gz_plugin::GaitAnimator")
