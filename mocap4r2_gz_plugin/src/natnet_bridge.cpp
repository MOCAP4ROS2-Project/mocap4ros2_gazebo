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

#include "mocap4r2_gz_plugin/natnet_bridge.hpp"

namespace mocap4r2_gz_plugin
{

namespace natnet = mocap4r2_optitrack_driver::natnet;

namespace
{

// Z-up to Motive Y-up: inverse of (x, y, z) -> (x, -z, y), i.e. (x, y, z) -> (x, z, -y).
std::array<float, 3> to_y_up(double x, double y, double z, bool y_up)
{
  return y_up ? std::array<float, 3>{static_cast<float>(x), static_cast<float>(z),
    static_cast<float>(-y)} :
         std::array<float, 3>{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}

}  // namespace

natnet::DataDescriptions
natnet_descriptions(const SceneConfig & config)
{
  natnet::DataDescriptions d;
  for (size_t i = 0; i < config.bodies.size(); i++) {
    natnet::RigidBodyDescription rb;
    rb.name = config.bodies[i].name;
    rb.id = static_cast<int32_t>(i + 1);
    rb.marker_names = config.bodies[i].marker_names;
    d.rigid_bodies.push_back(rb);
  }
  return d;
}

natnet::FrameData
to_natnet(const mocap4r2_control::MocapFrame & frame, bool y_up, double time)
{
  natnet::FrameData f;
  f.frame_number = static_cast<int32_t>(frame.frame_number);
  f.timestamp = time;
  for (const auto & rb : frame.rigid_bodies) {
    natnet::RigidBodyData d;
    d.id = rb.id;
    d.position = to_y_up(rb.pose.position.x, rb.pose.position.y, rb.pose.position.z, y_up);
    const auto v = to_y_up(rb.pose.orientation.x, rb.pose.orientation.y, rb.pose.orientation.z,
        y_up);
    d.orientation = {v[0], v[1], v[2], static_cast<float>(rb.pose.orientation.w)};
    d.mean_error = rb.mean_error > 0.0f ? rb.mean_error : 0.0f;
    d.params = rb.tracking_valid ? natnet::RB_TRACKING_VALID : 0;
    f.rigid_bodies.push_back(d);
    for (size_t k = 0; k < rb.markers.size(); k++) {
      const auto & m = rb.markers[k];
      natnet::LabeledMarker lm;
      lm.id = (rb.id << 16) | static_cast<int32_t>(k + 1);
      lm.position = to_y_up(m.position.x, m.position.y, m.position.z, y_up);
      lm.size = 0.014f;
      lm.params = m.occluded ? natnet::MARKER_OCCLUDED : 0;
      lm.residual = m.residual > 0.0f ? m.residual : 0.0f;
      f.labeled_markers.push_back(lm);
    }
  }
  return f;
}

}  // namespace mocap4r2_gz_plugin
