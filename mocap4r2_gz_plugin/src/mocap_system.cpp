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

// Gazebo system plugin: a simulated motion capture room. It reads the world pose
// of the configured links, adds markers, and streams frames either as a
// MOCAP4ROS2 system (output "ros") or as an OptiTrack Motive server (output
// "natnet") that the real OptiTrack driver connects to.

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "mocap4r2_control/frame_publisher.hpp"
#include "mocap4r2_control/mocap_system_node.hpp"
#include "mocap4r2_gz_plugin/mocap_scene.hpp"
#include "mocap4r2_gz_plugin/natnet_bridge.hpp"
#include "mocap4r2_optitrack_driver/fake_natnet_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_gz_plugin
{

// MOCAP4ROS2 system living inside Gazebo.
class GzMocapNode : public mocap4r2_control::MocapSystemNode
{
public:
  GzMocapNode(const std::string & name, double rate, const rclcpp::NodeOptions & options)
  : MocapSystemNode(name, "gazebo", options), rate_(rate)
  {
    mocap4r2_control::declare_frame_publisher_parameters(*this);
  }

  void publish(mocap4r2_control::MocapFrame && f)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (publisher_) {
      publisher_->publish(std::move(f));
    }
  }

protected:
  CallbackReturn on_system_configure() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publisher_ = std::make_unique<mocap4r2_control::FramePublisher>(*this);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_system_cleanup() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }
  double nominal_frame_rate() const override {return rate_;}
  std::string server() const override {return "Gazebo simulation";}
  bool connected() const override {return true;}

private:
  double rate_;
  std::mutex mutex_;
  std::unique_ptr<mocap4r2_control::FramePublisher> publisher_;
};

class MocapSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  ~MocapSystem() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (node_) {
      node_->shutdown();
    }
  }

  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    SceneConfig config;
    rate_ = get(sdf, "rate", 120.0);
    config.noise = get(sdf, "noise", 0.0);
    config.dropout = get(sdf, "dropout", 0.0);
    output_ = get(sdf, "output", std::string("ros"));
    auto elem = sdf->FindElement("rigid_body");
    while (elem) {
      BodyConfig b;
      b.name = elem->Get<std::string>("name");
      b.link = elem->Get<std::string>("link");
      if (elem->HasElement("offset")) {
        const auto o = elem->Get<gz::math::Pose3d>("offset");
        b.offset.p = {o.Pos().X(), o.Pos().Y(), o.Pos().Z()};
        b.offset.q = {o.Rot().X(), o.Rot().Y(), o.Rot().Z(), o.Rot().W()};
      }
      auto m = elem->FindElement("marker");
      int k = 0;
      while (m) {
        const auto v = m->Get<gz::math::Vector3d>();
        b.markers.push_back({v.X(), v.Y(), v.Z()});
        b.marker_names.push_back(
          m->HasAttribute("name") ? m->Get<std::string>("name") : "m" + std::to_string(++k));
        m = m->GetNextElement("marker");
      }
      if (b.name.empty() || b.link.find("::") == std::string::npos) {
        gzerr << "mocap4r2: rigid_body needs name and link=\"model::link\"" << std::endl;
      } else {
        config.bodies.push_back(b);
      }
      elem = elem->GetNextElement("rigid_body");
    }
    auto sk = sdf->FindElement("skeleton");
    while (sk) {
      SkeletonConfig sc;
      sc.name = sk->Get<std::string>("name");
      auto seg = sk->FindElement("segment");
      while (seg) {
        sc.segments.push_back(
          {seg->Get<std::string>("name"), seg->Get<std::string>("link"),
            seg->HasAttribute("parent") ? seg->Get<std::string>("parent") : ""});
        seg = seg->GetNextElement("segment");
      }
      config.skeletons.push_back(sc);
      sk = sk->GetNextElement("skeleton");
    }
    scene_ = std::make_unique<MocapScene>(config);

    if (output_ == "natnet") {
      mocap4r2_optitrack_driver::FakeNatNetServer::Config c;
      c.address = get(sdf, "natnet_address", std::string("127.0.0.1"));
      c.command_port = static_cast<uint16_t>(get(sdf, "natnet_command_port", 1510));
      c.data_port = static_cast<uint16_t>(get(sdf, "natnet_data_port", 1511));
      c.multicast = get(sdf, "natnet_multicast", false);
      y_up_ = get(sdf, "up_axis", std::string("y")) == "y";
      natnet_ = std::make_unique<mocap4r2_optitrack_driver::FakeNatNetServer>(
        c, natnet_descriptions(config));
      gzmsg << "mocap4r2: simulated Motive on " << c.address << ":" << c.command_port << std::endl;
    } else {
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);  // only if no other plugin did it
      }
      const auto name = get(sdf, "node_name", std::string("gz_mocap"));
      const auto ns = get(sdf, "namespace", std::string(""));
      const auto frame = get(sdf, "frame_id", std::string("mocap"));
      const auto autostart = get(sdf, "autostart", std::string("configure"));
      node_ = std::make_shared<GzMocapNode>(
        name, rate_, rclcpp::NodeOptions()
        .arguments({"--ros-args", "-r", "__ns:=/" + ns})
        .parameter_overrides({{"frame_id", frame}, {"publish_tf", get(sdf, "publish_tf", false)}}));
      using lifecycle_msgs::msg::Transition;
      if (autostart == "configure" || autostart == "activate") {
        node_->trigger_transition(Transition::TRANSITION_CONFIGURE);
      }
      if (autostart == "activate") {
        node_->trigger_transition(Transition::TRANSITION_ACTIVATE);
      }
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_->get_node_base_interface());
      spin_thread_ = std::thread([this] {executor_->spin();});
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info, const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || !scene_) {
      return;
    }
    const auto sim = std::chrono::duration<double>(info.simTime).count();
    if (frame_number_ > 0 && sim < next_time_) {
      return;
    }
    next_time_ = sim + 1.0 / rate_;
    if (sim < last_time_) {
      frame_number_ = 0;  // simulation reset
    }
    last_time_ = sim;

    std::map<std::string, Pose3> poses;
    std::vector<std::string> links;
    for (const auto & b : scene_->config().bodies) {
      links.push_back(b.link);
    }
    for (const auto & sk : scene_->config().skeletons) {
      for (const auto & seg : sk.segments) {
        links.push_back(seg.link);
      }
    }
    for (const auto & name : links) {
      const auto e = link_entity(name, ecm);
      if (e == gz::sim::kNullEntity) {
        continue;
      }
      const auto p = gz::sim::worldPose(e, ecm);
      Pose3 out;
      out.p = {p.Pos().X(), p.Pos().Y(), p.Pos().Z()};
      out.q = {p.Rot().X(), p.Rot().Y(), p.Rot().Z(), p.Rot().W()};
      poses[name] = out;
    }
    auto frame = scene_->frame(++frame_number_, poses);

    if (natnet_) {
      auto f = to_natnet(frame, y_up_, sim);
      const uint64_t ticks = static_cast<uint64_t>(sim * 1e7);
      f.camera_mid_exposure = ticks;
      f.camera_data_received = ticks;
      f.transmit = ticks;
      natnet_->send_frame(f);
    } else if (node_) {
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      frame.capture_time = rclcpp::Time(ns, RCL_ROS_TIME);
      node_->publish(std::move(frame));
    }
  }

private:
  template<typename T>
  static T get(const std::shared_ptr<const sdf::Element> & sdf, const std::string & key, T def)
  {
    return sdf->HasElement(key) ? sdf->Get<T>(key) : def;
  }

  gz::sim::Entity link_entity(const std::string & full, const gz::sim::EntityComponentManager & ecm)
  {
    const auto cached = links_.find(full);
    if (cached != links_.end() && cached->second != gz::sim::kNullEntity) {
      return cached->second;
    }
    const auto sep = full.find("::");
    const auto model = ecm.EntityByComponents(
      gz::sim::components::Name(full.substr(0, sep)), gz::sim::components::Model());
    gz::sim::Entity link = gz::sim::kNullEntity;
    if (model != gz::sim::kNullEntity) {
      link = gz::sim::Model(model).LinkByName(ecm, full.substr(sep + 2));
    }
    links_[full] = link;
    return link;
  }

  std::unique_ptr<MocapScene> scene_;
  std::string output_;
  double rate_{120.0};
  bool y_up_{true};
  double next_time_{0.0};
  double last_time_{0.0};
  uint64_t frame_number_{0};
  std::map<std::string, gz::sim::Entity> links_;
  std::shared_ptr<GzMocapNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<mocap4r2_optitrack_driver::FakeNatNetServer> natnet_;
};

}  // namespace mocap4r2_gz_plugin

GZ_ADD_PLUGIN(
  mocap4r2_gz_plugin::MocapSystem, gz::sim::System,
  mocap4r2_gz_plugin::MocapSystem::ISystemConfigure,
  mocap4r2_gz_plugin::MocapSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(mocap4r2_gz_plugin::MocapSystem, "mocap4r2_gz_plugin::MocapSystem")
