// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_
#define OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/type_traits/requires.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <utility>

namespace openscenario_interpreter
{
class SimulatorCore
{
public:  // TODO PRIVATE!
  static inline std::unique_ptr<traffic_simulator::API> connection = nullptr;

public:
  template <typename Node, typename... Ts>
  static auto activate(
    const Node & node, const traffic_simulator::Configuration & configuration, Ts &&... xs) -> void
  {
    if (not connection) {
      connection = std::make_unique<traffic_simulator::API>(node, configuration);
      connection->initialize(std::forward<decltype(xs)>(xs)...);
    } else {
      throw Error("The simulator core has already been instantiated.");
    }
  }

  static auto deactivate() -> void { connection.reset(); }

  static auto update() -> void { connection->updateFrame(); }

  class GeneralCommand  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    using NativeWorldPosition = geometry_msgs::msg::Pose;

    using NativeLanePosition = traffic_simulator_msgs::msg::LaneletPose;

    template <typename T, typename std::enable_if_t<std::is_same_v<T, NativeLanePosition>, int> = 0>
    static auto convert(const geometry_msgs::msg::Pose & pose)
    {
      if (const auto result = connection->toLaneletPose(pose, false); result) {
        return result.get();
      } else {
        throw Error(
          "The specified WorldPosition = [", pose.position.x, ", ", pose.position.y, ", ",
          pose.position.z,
          "] could not be approximated to the proper Lane. Perhaps the "
          "WorldPosition points to a location where multiple lanes overlap, and "
          "there are at least two or more candidates for a LanePosition that "
          "can be approximated to that WorldPosition. This issue can be "
          "resolved by strictly specifying the location using LanePosition "
          "instead of WorldPosition");
      }
    }

    template <
      typename T, typename std::enable_if_t<std::is_same_v<T, NativeWorldPosition>, int> = 0>
    static auto convert(const NativeLanePosition & native_lane_position)
    {
      return connection->toMapPose(native_lane_position);
    }

    template <typename OSCLanePosition>
    static auto makeNativeLanePosition(const OSCLanePosition & osc_lane_position)
    {
      return traffic_simulator::helper::constructLaneletPose(
        boost::lexical_cast<std::int64_t>(osc_lane_position.lane_id), osc_lane_position.s,
        osc_lane_position.offset, osc_lane_position.orientation.r, osc_lane_position.orientation.p,
        osc_lane_position.orientation.h);
    }

    template <typename OSCWorldPosition>
    static auto makeNativeWorldPosition(const OSCWorldPosition & osc_world_position)
    {
      NativeWorldPosition native_world_position;
      native_world_position.position.x = osc_world_position.x;
      native_world_position.position.y = osc_world_position.y;
      native_world_position.position.z = osc_world_position.z;
      native_world_position.orientation =
        quaternion_operation::convertEulerAngleToQuaternion([&]() {
          geometry_msgs::msg::Vector3 vector;
          vector.x = osc_world_position.r;
          vector.y = osc_world_position.p;
          vector.z = osc_world_position.h;
          return vector;
        }());
      return native_world_position;
    }

    template <typename... Ts>
    static auto toWayIDs(Ts &&... xs) -> decltype(auto)
    {
      return connection->getTrafficRelationReferees(std::forward<decltype(xs)>(xs)...);
    }
  };

  class ActionApplication  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto applyAcquirePositionAction(Ts &&... xs)
    {
      return connection->requestAcquirePosition(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyAddEntityAction(Ts &&... xs)
    {
      return connection->spawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename Controller>
    static auto applyAssignControllerAction(
      const std::string & entity_ref, Controller && controller) -> void
    {
      connection->setVelocityLimit(
        entity_ref, controller.properties.template get<Double>(
                      "maxSpeed", std::numeric_limits<Double::value_type>::max()));

      connection->setDriverModel(entity_ref, [&]() {
        auto message = connection->getDriverModel(entity_ref);
        message.see_around = not controller.properties.template get<Boolean>("isBlind");
        return message;
      }());

      if (controller.isUserDefinedController()) {
        connection->attachLidarSensor(entity_ref);

        connection->attachDetectionSensor([&]() {
          simulation_api_schema::DetectionSensorConfiguration configuration;
          configuration.set_entity(entity_ref);
          configuration.set_architecture_type(
            getParameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_update_duration(0.1);
          configuration.set_range(300);
          configuration.set_filter_by_range(
            controller.properties.template get<Boolean>("isClairvoyant"));
          return configuration;
        }());

        connection->attachOccupancyGridSensor([&]() {
          simulation_api_schema::OccupancyGridSensorConfiguration configuration;
          configuration.set_entity(entity_ref);
          configuration.set_architecture_type(
            getParameter<std::string>("architecture_type", "awf/universe"));
          configuration.set_update_duration(0.1);
          configuration.set_resolution(0.5);
          configuration.set_width(200);
          configuration.set_height(200);
          configuration.set_range(300);
          configuration.set_filter_by_range(
            controller.properties.template get<Boolean>("isClairvoyant"));
          return configuration;
        }());
      }
    }

    template <typename... Ts>
    static auto applyAssignRouteAction(Ts &&... xs)
    {
      return connection->requestAssignRoute(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyDeleteEntityAction(Ts &&... xs)
    {
      return connection->despawn(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyLaneChangeAction(Ts &&... xs)
    {
      return connection->requestLaneChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applySpeedAction(Ts &&... xs)
    {
      return connection->requestSpeedChange(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyTeleportAction(Ts &&... xs)
    {
      return connection->setEntityStatus(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto applyWalkStraightAction(Ts &&... xs)
    {
      return connection->requestWalkStraight(std::forward<decltype(xs)>(xs)...);
    }
  };

  class ConditionEvaluation  // OpenSCENARIO 1.1.1 Section 3.1.5
  {
  protected:
    template <typename... Ts>
    static auto evaluateAcceleration(Ts &&... xs)
    {
      return connection->getEntityStatus(std::forward<decltype(xs)>(xs)...)
        .action_status.accel.linear.x;
    }

    template <typename... Ts>
    static auto evaluateCollisionCondition(Ts &&... xs) -> bool
    {
      return connection->checkCollision(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateSimulationTime(Ts &&... xs) -> double
    {
      return connection->getCurrentTime(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateSpeed(Ts &&... xs)
    {
      return connection->getEntityStatus(std::forward<decltype(xs)>(xs)...)
        .action_status.twist.linear.x;
    }

    template <typename... Ts>
    static auto evaluateStandStill(Ts &&... xs)
    {
      if (const auto result = connection->getStandStillDuration(std::forward<decltype(xs)>(xs)...);
          result) {
        return result.get();
      } else {
        using value_type = typename std::decay<decltype(result)>::type::value_type;
        return std::numeric_limits<value_type>::quiet_NaN();
      }
    }

    template <typename... Ts>
    static auto evaluateTimeHeadway(Ts &&... xs)
    {
      if (const auto result = connection->getTimeHeadway(std::forward<decltype(xs)>(xs)...);
          result) {
        return result.get();
      } else {
        using value_type = typename std::decay<decltype(result)>::type::value_type;
        return std::numeric_limits<value_type>::quiet_NaN();
      }
    }
  };

  class NonStandardOperation
  {
  protected:
    template <typename Performance, typename Properties>
    static auto activatePerformanceAssertion(
      const std::string & entity_ref, const Performance & performance,
      const Properties & properties)
    {
      connection->addMetric<metrics::OutOfRangeMetric>(entity_ref + "-out-of-range", [&]() {
        metrics::OutOfRangeMetric::Config configuration;
        configuration.target_entity = entity_ref;
        configuration.min_velocity = -performance.max_speed;
        configuration.max_velocity = +performance.max_speed;
        configuration.min_acceleration = -performance.max_deceleration;
        configuration.max_acceleration = +performance.max_acceleration;
        configuration.min_jerk = properties.template get<Double>("minJerk", Double::lowest());
        configuration.max_jerk = properties.template get<Double>("maxJerk", Double::max());
        configuration.jerk_topic =
          "/planning/scenario_planning/motion_velocity_optimizer/closest_jerk";
        return configuration;
      }());
    }

    template <typename... Ts>
    static auto asAutoware(Ts &&... xs) -> decltype(auto)
    {
      return connection->asAutoware(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    static auto evaluateCurrentState(Ts &&... xs) -> decltype(auto)
    {
      return connection->getCurrentAction(std::forward<decltype(xs)>(xs)...);
    }
  };
};

template <typename... Ts>
auto getRelativePose(Ts &&... xs)
try {
  return SimulatorCore::connection->getRelativePose(std::forward<decltype(xs)>(xs)...);
} catch (...) {
  geometry_msgs::msg::Pose result{};
  result.position.x = std::numeric_limits<double>::quiet_NaN();
  result.position.y = std::numeric_limits<double>::quiet_NaN();
  result.position.z = std::numeric_limits<double>::quiet_NaN();
  result.orientation.x = 0;
  result.orientation.y = 0;
  result.orientation.z = 0;
  result.orientation.w = 1;
  return result;
}

#define STRIP_OPTIONAL(IDENTIFIER, ALTERNATE)                                                     \
  template <typename... Ts>                                                                       \
  auto IDENTIFIER(Ts &&... xs)                                                                    \
  {                                                                                               \
    const auto result = SimulatorCore::connection->IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
    if (result) {                                                                                 \
      return result.get();                                                                        \
    } else {                                                                                      \
      using value_type = typename std::decay<decltype(result)>::type::value_type;                 \
      return ALTERNATE;                                                                           \
    }                                                                                             \
  }                                                                                               \
  static_assert(true, "")

STRIP_OPTIONAL(getBoundingBoxDistance, static_cast<value_type>(0));
STRIP_OPTIONAL(getLongitudinalDistance, std::numeric_limits<value_type>::quiet_NaN());

#undef STRIP_OPTIONAL
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SIMULATOR_CORE_HPP_