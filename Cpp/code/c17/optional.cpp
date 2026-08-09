/*
 * @Author: JohnJeep
 * @Date: 2026-08-09 10:35:29
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-08-09 23:08:17
 * @Description: std::optional is a wrapper that contains an optional value;
 *               it may contain a value or it may be empty.
 *               It is a type-safe way of representing optional values, and it can be used to
 *               indicate the absence of a value without resorting to null pointers or sentinel values.
 *               std::optional is part of the C++17 standard library,
 *               and is defined in the <optional> header.
 *               It provides a convenient way to handle cases where a value may or may not be present,
 *               and it can * help improve code clarity and safety.
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
 */
#include <iostream>
#include <optional>

std::optional<int> getValue(bool condition)
{
  if (condition) {
    return 42;
  }
  return std::nullopt;  // Return an empty optional if the condition is false
}

int main(int argc, char* argv[])
{
  std::optional<int> result = getValue(false);
  if (result.has_value()) {
    std::cout << "Value: " << result.value() << std::endl;
  } else {
    std::cout << "No value returned." << std::endl;
    try {
      result.value();  // This will throw std::bad_optional_access exception
    } catch (const std::bad_optional_access& e) {
      std::cout << "Exception caught: " << e.what() << std::endl;
    }
    // no value may set default value
    int val = result.value_or(0);  // This will return 0 if result is empty
    std::cout << "Value with default: " << val << std::endl;
  }

  return 0;
}
