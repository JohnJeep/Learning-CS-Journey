/*
 * @Author: JohnJeep
 * @Date: 2021-05-15 18:23:25
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-08-09 17:56:17
 * @Description: for_each usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;

class PrintInt {
 public:
  PrintInt(/* args */) {}

  ~PrintInt() {}

  void operator()(int elem) const { cout << elem << " "; }
};

int main(int argc, char* argv[])
{
  vector<int> coll;

  for (int i = 1; i <= 9; ++i) {
    coll.push_back(i);
  }

  for_each(coll.begin(), coll.end(), PrintInt());  // PrintInt() 是一个 function object
  cout << endl;

  return 0;
}
