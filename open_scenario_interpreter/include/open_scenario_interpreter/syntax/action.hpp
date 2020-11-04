// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_

#include <open_scenario_interpreter/syntax/global_action.hpp>
#include <open_scenario_interpreter/syntax/private_action.hpp>
#include <open_scenario_interpreter/syntax/storyboard_element.hpp>
#include <open_scenario_interpreter/syntax/user_defined_action.hpp>

#include <utility>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== Action ===============================================================
 *
 * <xsd:complexType name="Action">
 *   <xsd:choice>
 *     <xsd:element name="GlobalAction" type="GlobalAction"/>
 *     <xsd:element name="UserDefinedAction" type="UserDefinedAction"/>
 *     <xsd:element name="PrivateAction" type="PrivateAction"/>
 *   </xsd:choice>
 *   <xsd:attribute name="name" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Action
  : public StoryboardElement<Action>, public Element
{
  const String name;

  template<typename Node, typename Scope>
  explicit Action(const Node & node, Scope & scope, std::size_t maximum_execution_count)
  : StoryboardElement(maximum_execution_count),
    name(readAttribute<String>("name", node, scope))
  {
    choice(node,
      std::make_pair("GlobalAction", [&](auto && node) {
        return rebind<GlobalAction>(node, scope);
      }),
      std::make_pair("UserDefinedAction", [&](auto && node) {
        return rebind<UserDefinedAction>(node, scope);
      }),
      std::make_pair("PrivateAction", [&](auto && node) {
        return rebind<PrivateAction>(node, scope);
      }));
  }

  auto ready() const
  {
    return static_cast<bool>(*this);
  }

  static constexpr auto stopTriggered() noexcept
  {
    return false;
  }

  using Element::start;

  /* -------------------------------------------------------------------------
   *
   * Action
   *   An Action's goal is a function of the Action type and cannot be
   *   generalized. Accomplishing an Action's goal will involve meeting some
   *   arbitrary prerequisites related with the Action type (for example, a
   *   SpeedAction accomplishes its goal when the considered Entity is
   *   travelling at the prescribed speed). If an Action is acting on an
   *   EntitySelection, all instances of Entity within the selection have to
   *   complete in order to reach the completeState of the Action.
   *
   * ---------------------------------------------------------------------- */
  using Element::accomplished;

  using StoryboardElement::evaluate;

  Boolean overridden {false};

  void stop()
  {
    if (overridden) {
      current_state = complete_state;
    } else {
      overridden = true;
    }
  }

  template<typename ... Ts>
  decltype(auto) run(Ts && ... xs)
  {
    return Element::evaluate(std::forward<decltype(xs)>(xs)...);
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__ACTION_HPP_