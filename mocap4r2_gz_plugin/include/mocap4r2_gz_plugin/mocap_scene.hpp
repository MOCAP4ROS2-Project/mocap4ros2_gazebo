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

#ifndef MOCAP4R2_GZ_PLUGIN__MOCAP_SCENE_HPP_
#define MOCAP4R2_GZ_PLUGIN__MOCAP_SCENE_HPP_

#include <array>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "mocap4r2_control/frame_publisher.hpp"

namespace mocap4r2_gz_plugin
{

struct Pose3
{
  std::array<double, 3> p{0.0, 0.0, 0.0};
  std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
};

Pose3 compose(const Pose3 & a, const Pose3 & b);

// A tracked body: a link of the world and markers attached to it.
struct BodyConfig
{
  std::string name;
  std::string link;                          // "model::link"
  Pose3 offset;                              // body frame in the link frame
  std::vector<std::string> marker_names;
  std::vector<std::array<double, 3>> markers;  // in the body frame
};

// An articulated body: one segment per link, with its parent segment.
struct SkeletonConfig
{
  struct SegmentConfig
  {
    std::string name;
    std::string link;    // "model::link"
    std::string parent;  // name of the parent segment, empty for the root
  };
  std::string name;
  std::vector<SegmentConfig> segments;
};

struct SceneConfig
{
  std::vector<BodyConfig> bodies;
  std::vector<SkeletonConfig> skeletons;
  double noise{0.0};               // marker noise std, metres
  double dropout{0.0};             // probability of losing a body in a frame
  unsigned int seed{1};
};

// Builds mocap frames from link poses, as a real mocap system would see them.
class MocapScene
{
public:
  explicit MocapScene(const SceneConfig & config);

  // link_poses: world pose of each link by "model::link"; missing links are
  // reported as untracked bodies.
  mocap4r2_control::MocapFrame frame(
    uint64_t frame_number, const std::map<std::string, Pose3> & link_poses);

  const SceneConfig & config() const {return config_;}

private:
  SceneConfig config_;
  std::mt19937 rng_;
};

}  // namespace mocap4r2_gz_plugin

#endif  // MOCAP4R2_GZ_PLUGIN__MOCAP_SCENE_HPP_
