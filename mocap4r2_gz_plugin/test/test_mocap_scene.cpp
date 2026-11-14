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

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>

#include "mocap4r2_gz_plugin/mocap_scene.hpp"
#include "mocap4r2_gz_plugin/natnet_bridge.hpp"
#include "mocap4r2_optitrack_driver/frame_converter.hpp"

using mocap4r2_gz_plugin::BodyConfig;
using mocap4r2_gz_plugin::MocapScene;
using mocap4r2_gz_plugin::Pose3;
using mocap4r2_gz_plugin::SceneConfig;
namespace natnet = mocap4r2_optitrack_driver::natnet;

namespace
{

SceneConfig config()
{
  SceneConfig c;
  BodyConfig b;
  b.name = "robot";
  b.link = "robot::base_link";
  b.offset.p = {0.0, 0.0, 0.5};
  b.markers = {{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}};
  b.marker_names = {"front", "left"};
  c.bodies = {b};
  BodyConfig w = b;
  w.name = "wand";
  w.link = "wand::link";
  c.bodies.push_back(w);
  return c;
}

Pose3 yawed(double x, double y, double yaw)
{
  Pose3 p;
  p.p = {x, y, 0.0};
  p.q = {0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)};
  return p;
}

}  // namespace

TEST(MocapScene, body_pose_and_markers_follow_the_link)
{
  MocapScene scene(config());
  const auto f = scene.frame(7, {{"robot::base_link", yawed(1.0, 2.0, M_PI_2)}});
  EXPECT_EQ(f.frame_number, 7u);
  ASSERT_EQ(f.rigid_bodies.size(), 2u);
  const auto & rb = f.rigid_bodies[0];
  EXPECT_TRUE(rb.tracking_valid);
  EXPECT_NEAR(rb.pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(rb.pose.position.z, 0.5, 1e-12);
  ASSERT_EQ(rb.markers.size(), 2u);
  // "front" (0.1, 0, 0) rotated 90 degrees: +y
  EXPECT_NEAR(rb.markers[0].position.x, 1.0, 1e-12);
  EXPECT_NEAR(rb.markers[0].position.y, 2.1, 1e-12);
  EXPECT_EQ(rb.markers[0].name, "robot:front");
  // The link of the wand is not in the world: untracked, no markers.
  EXPECT_FALSE(f.rigid_bodies[1].tracking_valid);
  EXPECT_TRUE(f.rigid_bodies[1].markers.empty());
  EXPECT_EQ(f.markers.size(), 2u);
}

TEST(MocapScene, skeletons_from_links)
{
  auto c = config();
  mocap4r2_gz_plugin::SkeletonConfig sk;
  sk.name = "walker";
  sk.segments = {{"pelvis", "walker::pelvis", ""}, {"thigh", "walker::thigh", "pelvis"},
    {"shank", "walker::shank", "thigh"}};
  c.skeletons = {sk};
  MocapScene scene(c);
  auto f = scene.frame(
    3, {{"walker::pelvis", yawed(0, 0, 0.5)}, {"walker::thigh", yawed(0.1, 0, 0.5)}});
  ASSERT_EQ(f.skeletons.size(), 1u);
  const auto & s = f.skeletons[0];
  EXPECT_EQ(s.name, "walker");
  ASSERT_EQ(s.segments.size(), 3u);
  EXPECT_EQ(s.segments[0].parent_id, mocap4r2_interfaces::msg::Segment::NO_PARENT);
  EXPECT_EQ(s.segments[1].parent_id, s.segments[0].id);
  EXPECT_EQ(s.segments[2].parent_id, s.segments[1].id);
  EXPECT_NEAR(s.segments[1].pose.position.x, 0.1, 1e-12);
  EXPECT_FALSE(s.segments[2].tracking_valid);  // link missing
  EXPECT_FALSE(s.tracking_valid);
}

TEST(MocapScene, noise_and_dropout)
{
  auto c = config();
  c.noise = 0.01;
  c.dropout = 1.0;
  MocapScene scene(c);
  const auto f = scene.frame(1, {{"robot::base_link", yawed(0, 0, 0)}});
  EXPECT_FALSE(f.rigid_bodies[0].tracking_valid);
  EXPECT_TRUE(f.rigid_bodies[0].markers[0].occluded);
  c.dropout = 0.0;
  MocapScene a(c), b(c);
  const auto fa = a.frame(1, {{"robot::base_link", yawed(0, 0, 0)}});
  const auto fb = b.frame(1, {{"robot::base_link", yawed(0, 0, 0)}});
  EXPECT_EQ(fa.markers[0].position.x, fb.markers[0].position.x);  // same seed
  EXPECT_NE(fa.markers[0].position.x, 0.1);                        // noise applied
}

// The OptiTrack driver connected to the simulated Motive sees the same scene.
TEST(MocapScene, natnet_round_trip_through_the_optitrack_converter)
{
  MocapScene scene(config());
  const auto f = scene.frame(
    42, {{"robot::base_link", yawed(1.0, -2.0, 0.7)}, {"wand::link", yawed(0.0, 1.0, -1.2)}});
  for (bool y_up : {true, false}) {
    const auto nf = mocap4r2_gz_plugin::to_natnet(f, y_up, 1.0);
    const auto bytes = natnet::encode_frame(nf, {4, 4});
    const auto parsed = natnet::parse_frame(bytes.data(), bytes.size(), {4, 4});
    mocap4r2_optitrack_driver::ConverterOptions o;
    o.up_axis = y_up ? mocap4r2_optitrack_driver::UpAxis::Y : mocap4r2_optitrack_driver::UpAxis::Z;
    const auto model = mocap4r2_optitrack_driver::Model::from(
      mocap4r2_gz_plugin::natnet_descriptions(config()));
    const auto back = mocap4r2_optitrack_driver::convert_frame(parsed, model, o);
    ASSERT_EQ(back.rigid_bodies.size(), 2u);
    for (size_t i = 0; i < 2; i++) {
      const auto & a = f.rigid_bodies[i].pose;
      const auto & b = back.rigid_bodies[i].pose;
      EXPECT_EQ(back.rigid_bodies[i].name, f.rigid_bodies[i].name);
      EXPECT_NEAR(a.position.x, b.position.x, 1e-5);
      EXPECT_NEAR(a.position.y, b.position.y, 1e-5);
      EXPECT_NEAR(a.position.z, b.position.z, 1e-5);
      EXPECT_NEAR(std::abs(a.orientation.x * b.orientation.x + a.orientation.y * b.orientation.y +
        a.orientation.z * b.orientation.z + a.orientation.w * b.orientation.w), 1.0, 1e-5);
    }
    ASSERT_EQ(back.markers.size(), 4u);
    EXPECT_EQ(back.markers[1].name, "robot:left");
    EXPECT_NEAR(back.markers[1].position.y, f.markers[1].position.y, 1e-5);
  }
}
