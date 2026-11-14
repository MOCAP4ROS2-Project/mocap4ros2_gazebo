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

#ifndef MOCAP4R2_GZ_PLUGIN__NATNET_BRIDGE_HPP_
#define MOCAP4R2_GZ_PLUGIN__NATNET_BRIDGE_HPP_

#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_gz_plugin/mocap_scene.hpp"
#include "mocap4r2_optitrack_driver/natnet_protocol.hpp"

namespace mocap4r2_gz_plugin
{

// Data descriptions announced by the simulated Motive.
mocap4r2_optitrack_driver::natnet::DataDescriptions natnet_descriptions(
  const SceneConfig & config);

// Frame as Motive would stream it (Y-up, its default, if y_up).
mocap4r2_optitrack_driver::natnet::FrameData to_natnet(
  const mocap4r2_control::MocapFrame & frame, bool y_up, double time);

}  // namespace mocap4r2_gz_plugin

#endif  // MOCAP4R2_GZ_PLUGIN__NATNET_BRIDGE_HPP_
