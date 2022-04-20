// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <simple_noise_simulator/exception.hpp>
#include <simple_noise_simulator/simple_noise_simulator.hpp>
#include <simulation_interface/conversions.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace simple_noise_simulator
{
ScenarioSimulator::ScenarioSimulator(const rclcpp::NodeOptions & options)
: Node("simple_noise_simulator", options)
{

  // perception noise
  {
    std::random_device seed;
    auto & m = perception_noise_;
    m.rand_engine_ = std::make_shared<std::mt19937>(seed());
    float64_t pos_noise_stddev = declare_parameter("pos_noise_stddev", 0.1);
    float64_t vel_noise_stddev = declare_parameter("vel_noise_stddev", 1e-2);
    float64_t rpy_noise_stddev = declare_parameter("rpy_noise_stddev", 1e-4);
    float64_t pos_delay_time = declare_parameter("pos_delay_time", 0.5);
    float64_t twist_delay_time = declare_parameter("twist_delay_time", 0.0);
    float64_t lost_probability = declare_parameter("lost_probability", 0.0);
    m.pos_noise_dist_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    m.vel_noise_dist_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    m.rpy_noise_dist_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    m.pos_delay_time_ = std::make_shared<float64_t>(pos_delay_time);
    m.vel_delay_time_ = std::make_shared<float64_t>(twist_delay_time);
    m.lost_prob_ = std::make_shared<float64_t>(lost_probability);

    // x_stddev_ = declare_parameter("x_stddev", 0.0001);
    // y_stddev_ = declare_parameter("y_stddev", 0.0001);
  }

  sub_detected_objects_with_noise_ =
    create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "/perception/object_recognition/detection/objects", rclcpp::QoS{1},
      std::bind(&ScenarioSimulator::updateEntityStatusWithNoise, this, std::placeholders::_1));
  pub_detected_objects_with_noise_ =
     create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
       "/perception/object_recognition/detection/objects_with_noise", rclcpp::QoS{1});

  
  

}

ScenarioSimulator::~ScenarioSimulator() {}

void ScenarioSimulator::updateEntityStatusWithNoise(
  const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  detected_objects_ptr_ = msg;
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
  


  auto & n = perception_noise_;
  for (auto & object : detected_objects_ptr_->objects) {
    RCLCPP_ERROR(rclcpp::get_logger("noise_simulator"), "before noise %lf", object.kinematics.pose_with_covariance.pose.position.x);
    object.kinematics.pose_with_covariance.pose.position.x += (*n.pos_noise_dist_)(*n.rand_engine_);
    object.kinematics.pose_with_covariance.pose.position.y += (*n.pos_noise_dist_)(*n.rand_engine_);
    RCLCPP_ERROR(rclcpp::get_logger("noise_simulator"), "after noise %lf",object.kinematics.pose_with_covariance.pose.position.x);

  }
  detected_objects.header = detected_objects_ptr_->header;
  detected_objects.objects = detected_objects_ptr_->objects;



  pub_detected_objects_with_noise_->publish(detected_objects);


  // auto & n = perception_noise_;
  // for (const auto & object : detected_objects_ptr_)
  // {
    
  // }
   
  // odom.pose.pose.position.x += (*n.pos_dist_)(*n.rand_engine_);
  // odom.pose.pose.position.y += (*n.pos_dist_)(*n.rand_engine_);
  // const auto velocity_noise = (*n.vel_dist_)(*n.rand_engine_);
  // odom.twist.twist.linear.x = velocity_noise;
  // float32_t yaw = motion::motion_common::to_angle(odom.pose.pose.orientation);
  // yaw += static_cast<float>((*n.rpy_dist_)(*n.rand_engine_));
  // odom.pose.pose.orientation = motion::motion_common::from_angle(yaw);

  // vel.longitudinal_velocity += static_cast<float32_t>(velocity_noise);

  // steer.steering_tire_angle += static_cast<float32_t>((*n.steer_dist_)(*n.rand_engine_));

  // pub_detected_objects_with_noise_->publish(detected_objects_ptr_);
  // entity_status_ = {};
  // for (const auto proto : req.status()) {
  //   entity_status_.emplace_back(proto);
  // }
  // res = simulation_api_schema::UpdateEntityStatusResponse();
  // res.mutable_result()->set_success(true);
  // res.mutable_result()->set_description("");
}

// void ScenarioSimulator::add_perception_noise(
//   Odometry & odom, VelocityReport & vel, SteeringReport & steer) const
// {
//   auto & n = perception_noise_;
//   odom.pose.pose.position.x += (*n.pos_dist_)(*n.rand_engine_);
//   odom.pose.pose.position.y += (*n.pos_dist_)(*n.rand_engine_);
//   const auto velocity_noise = (*n.vel_dist_)(*n.rand_engine_);
//   odom.twist.twist.linear.x = velocity_noise;
//   float32_t yaw = motion::motion_common::to_angle(odom.pose.pose.orientation);
//   yaw += static_cast<float>((*n.rpy_dist_)(*n.rand_engine_));
//   odom.pose.pose.orientation = motion::motion_common::from_angle(yaw);

//   vel.longitudinal_velocity += static_cast<float32_t>(velocity_noise);

//   steer.steering_tire_angle += static_cast<float32_t>((*n.steer_dist_)(*n.rand_engine_));
}  // namespace simple_noise_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(simple_noise_simulator::ScenarioSimulator)
