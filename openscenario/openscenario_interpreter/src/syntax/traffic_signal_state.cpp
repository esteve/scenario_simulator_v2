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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
TrafficSignalState::TrafficSignalState(const pugi::xml_node & node, Scope & scope)
: traffic_signal_id(readAttribute<String>("trafficSignalId", node, scope)),
  state(readAttribute<String>("state", node, scope))
{
}

auto TrafficSignalState::evaluate() const -> Object
{
  setConventionalTrafficLightsState(id(), state);
  return unspecified;
}

auto TrafficSignalState::id() const -> lanelet::Id
{
  return boost::lexical_cast<lanelet::Id>(traffic_signal_id);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
