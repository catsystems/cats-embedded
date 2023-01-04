/*
 * CATS Flight Software
 * Copyright (C) 2022 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <utility>

namespace task {

template <typename T>
class Task {
 protected:
  /* Protected constructor */
  Task() = default;

  virtual void Run() = 0;

 public:
  /* Deleted move constructor & move assignment operator */
  Task(Task&&) = delete;
  Task& operator=(Task&&) = delete;

  /* Deleted copy constructor & assignment operator */
  Task(const Task&) = delete;
  Task& operator=(const Task&) = delete;

  /* Deleted new & delete operators */
  static void* operator new(std::size_t size) = delete;
  static void* operator new[](std::size_t size) = delete;
  static void operator delete(void* ptr) = delete;
  static void operator delete[](void* ptr) = delete;

  template <typename... Args>
  static T& GetInstance(Args&&... args) {
    /* Static local variable */
    static T instance(std::forward<Args>(args)...);
    return instance;
  }
};

}  // namespace task
