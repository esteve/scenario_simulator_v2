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

#ifndef CONCEALER__TASK_QUEUE_HPP_
#define CONCEALER__TASK_QUEUE_HPP_

#include <atomic>
#include <exception>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace concealer
{
class TaskQueue
{
  using Thunk = std::function<void()>;

  std::queue<Thunk> thunks;

  mutable std::mutex thunks_mutex;

  std::thread dispatcher;

  std::atomic<bool> finalized = false;

  std::exception_ptr thrown;

  auto front() const -> Thunk;

  auto pop() -> void;

public:
  explicit TaskQueue();

  ~TaskQueue();

  template <typename F>
  auto delay(F && f) -> void
  {
    rethrow();
    auto lock = std::unique_lock(thunks_mutex);
    thunks.emplace(std::forward<F>(f));
  }

  auto empty() const -> bool;

  auto rethrow() const -> void;
};
}  // namespace concealer

#endif  // CONCEALER__TASK_QUEUE_HPP_
