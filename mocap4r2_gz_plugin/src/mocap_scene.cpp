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

#include <map>
#include <string>
#include <vector>

#include "mocap4r2_gz_plugin/mocap_scene.hpp"

namespace mocap4r2_gz_plugin
{

Pose3
compose(const Pose3 & a, const Pose3 & b)
{
  const double ax = a.q[0], ay = a.q[1], az = a.q[2], aw = a.q[3];
  const double bx = b.q[0], by = b.q[1], bz = b.q[2], bw = b.q[3];
  Pose3 r;
  r.q = {aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz};
  const double px = b.p[0], py = b.p[1], pz = b.p[2];
  const double cx = ay * pz - az * py, cy = az * px - ax * pz, cz = ax * py - ay * px;
  r.p = {a.p[0] + px + 2.0 * (aw * cx + ay * cz - az * cy),
    a.p[1] + py + 2.0 * (aw * cy + az * cx - ax * cz),
    a.p[2] + pz + 2.0 * (aw * cz + ax * cy - ay * cx)};
  return r;
}

MocapScene::MocapScene(const SceneConfig & config)
: config_(config), rng_(config.seed)
{
}

mocap4r2_control::MocapFrame
MocapScene::frame(uint64_t frame_number, const std::map<std::string, Pose3> & link_poses)
{
  using mocap4r2_interfaces::msg::Marker;
  using mocap4r2_interfaces::msg::RigidBody;
  // std::normal_distribution requires stddev > 0; it is not sampled when noise is 0
  std::normal_distribution<double> noise(0.0, config_.noise > 0.0 ? config_.noise : 1.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  mocap4r2_control::MocapFrame out;
  out.frame_number = frame_number;
  for (size_t i = 0; i < config_.bodies.size(); i++) {
    const auto & b = config_.bodies[i];
    RigidBody rb;
    rb.id = static_cast<int32_t>(i + 1);
    rb.name = b.name;
    const auto link = link_poses.find(b.link);
    const bool lost = config_.dropout > 0.0 && unit(rng_) < config_.dropout;
    rb.tracking_valid = link != link_poses.end() && !lost;
    rb.mean_error = rb.tracking_valid ? static_cast<float>(config_.noise) : -1.0f;
    rb.pose.orientation.w = 1.0;
    if (link != link_poses.end()) {
      const Pose3 body = compose(link->second, b.offset);
      if (rb.tracking_valid) {
        rb.pose.position.x = body.p[0];
        rb.pose.position.y = body.p[1];
        rb.pose.position.z = body.p[2];
        rb.pose.orientation.x = body.q[0];
        rb.pose.orientation.y = body.q[1];
        rb.pose.orientation.z = body.q[2];
        rb.pose.orientation.w = body.q[3];
      }
      for (size_t k = 0; k < b.markers.size(); k++) {
        Pose3 local;
        local.p = b.markers[k];
        const Pose3 m = compose(body, local);
        Marker mk;
        mk.id = static_cast<int32_t>((i + 1) * 1000 + k + 1);
        mk.name = b.name + ":" + b.marker_names[k];
        const double n = config_.noise;
        mk.position.x = m.p[0] + (n > 0.0 ? noise(rng_) : 0.0);
        mk.position.y = m.p[1] + (n > 0.0 ? noise(rng_) : 0.0);
        mk.position.z = m.p[2] + (n > 0.0 ? noise(rng_) : 0.0);
        mk.occluded = lost;
        mk.residual = static_cast<float>(n);
        rb.markers.push_back(mk);
        out.markers.push_back(mk);
      }
    }
    out.rigid_bodies.push_back(rb);
  }

  for (size_t i = 0; i < config_.skeletons.size(); i++) {
    const auto & sc = config_.skeletons[i];
    mocap4r2_interfaces::msg::Skeleton sk;
    sk.id = static_cast<int32_t>(i + 1);
    sk.name = sc.name;
    sk.tracking_valid = !sc.segments.empty();
    std::map<std::string, int32_t> ids;
    for (size_t k = 0; k < sc.segments.size(); k++) {
      ids[sc.segments[k].name] = static_cast<int32_t>(k + 1);
    }
    for (const auto & seg : sc.segments) {
      mocap4r2_interfaces::msg::Segment s;
      s.id = ids[seg.name];
      s.name = seg.name;
      const auto parent = ids.find(seg.parent);
      s.parent_id = parent != ids.end() ? parent->second :
        mocap4r2_interfaces::msg::Segment::NO_PARENT;
      const auto link = link_poses.find(seg.link);
      s.tracking_valid = link != link_poses.end();
      s.pose.orientation.w = 1.0;
      if (s.tracking_valid) {
        s.pose.position.x = link->second.p[0];
        s.pose.position.y = link->second.p[1];
        s.pose.position.z = link->second.p[2];
        s.pose.orientation.x = link->second.q[0];
        s.pose.orientation.y = link->second.q[1];
        s.pose.orientation.z = link->second.q[2];
        s.pose.orientation.w = link->second.q[3];
      }
      sk.tracking_valid = sk.tracking_valid && s.tracking_valid;
      sk.segments.push_back(s);
    }
    out.skeletons.push_back(sk);
  }
  return out;
}

}  // namespace mocap4r2_gz_plugin
