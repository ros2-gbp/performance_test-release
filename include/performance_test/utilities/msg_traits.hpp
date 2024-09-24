// Copyright 2022-2024 Apex.AI, Inc.
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

#ifndef PERFORMANCE_TEST__UTILITIES__MSG_TRAITS_HPP_
#define PERFORMANCE_TEST__UTILITIES__MSG_TRAITS_HPP_

#include <type_traits>

namespace performance_test
{
struct MsgTraits
{
  // TODO(erik.snider) use concepts when upgrading to C++20

  template<typename T, typename = void>
  struct has_id_object : std::false_type {};

  template<typename T>
  struct has_id_object<
    T, std::void_t<decltype(std::declval<T>().id)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_id_function : std::false_type {};

  template<typename T>
  struct has_id_function<
    T, std::void_t<decltype(std::declval<T>().id())>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_time_object : std::false_type {};

  template<typename T>
  struct has_time_object<
    T, std::void_t<decltype(std::declval<T>().time)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_time_function : std::false_type {};

  template<typename T>
  struct has_time_function<
    T, std::void_t<decltype(std::declval<T>().time())>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_bounded_sequence : std::false_type {};

  template<typename T>
  struct has_bounded_sequence<
    T, std::void_t<decltype(std::declval<T>().bounded_sequence)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_bounded_sequence_func : std::false_type {};

  template<typename T>
  struct has_bounded_sequence_func<
    T, std::void_t<decltype(std::declval<T>().bounded_sequence())>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_sequence : std::false_type {};

  template<typename T>
  struct has_unbounded_sequence<
    T, std::void_t<decltype(std::declval<T>().unbounded_sequence)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_sequence_func : std::false_type {};

  template<typename T>
  struct has_unbounded_sequence_func<
    T, std::void_t<decltype(std::declval<T>().unbounded_sequence())>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_string : std::false_type {};

  template<typename T>
  struct has_unbounded_string<
    T, std::void_t<decltype(std::declval<T>().unbounded_string)>>
    : std::true_type {};

  template<typename T, typename = void>
  struct has_unbounded_string_func : std::false_type {};

  template<typename T>
  struct has_unbounded_string_func<
    T, std::void_t<decltype(std::declval<T>().unbounded_string())>>
    : std::true_type {};
};

}  // namespace performance_test

#endif  // PERFORMANCE_TEST__UTILITIES__MSG_TRAITS_HPP_
