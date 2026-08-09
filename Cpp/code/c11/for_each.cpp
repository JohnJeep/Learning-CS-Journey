/*
 * @Author: JohnJeep
 * @Date: 2021-05-15 18:23:25
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-08-09 18:16:35
 * @Description: for_each usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

class PrintInt {
 public:
  PrintInt(/* args */) {}

  ~PrintInt() {}

  void operator()(int elem) const { std::cout << elem << " "; }
};

int main(int argc, char* argv[])
{
  std::vector<int> coll;

  for (int i = 1; i <= 9; ++i) {
    coll.push_back(i);
  }

  std::for_each(coll.begin(), coll.end(), PrintInt());  // PrintInt() 是一个 function object
  std::cout << "\n";

  return 0;
}
