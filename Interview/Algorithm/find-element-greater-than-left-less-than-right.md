<!--
 * @Author: JohnJeep
 * @Date: 2021-09-07 22:15:35
 * @LastEditTime: 2021-09-07 22:32:03
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
-->

# 题目

找出数组中比左边的大比右边的小的元素


# 题目描述

一个整形数组，找出所有满足如下要求的数：所有它左边的数都比它小，所有右边的数都比它大。
例如：数组[1, 2, 4, 3, 9, 5, 6, 12, 15]，数组中l,2，12满足条件，时间复杂度要求小于等于O(Xn)，X为常数。



# 思路

定义一个辅助数组存右侧最小值，从右往左依次遍历，将当前值与后一个值的最小值存入辅助数组中；
然后再从左往右依次遍历，得到左侧的最大值，当这个值与辅助数组中最小值相等时，满足条；即左边最大等于右边最小时满足条件。


# 题解
```cpp
#include <iostream>
#include <vector>
using namespace std;

vector<int> func(int data[], int len)
{
    vector<int> vec;
    int* right_min = new int[len];       // 定义一个辅助数组存右侧最小值
    right_min[len - 1] = data[len - 1];  // 数组的最后一位存最右侧值，设初值

    for (int i = len - 2; i >= 0; --i) {
        right_min[i] = std::min(data[i], data[i+1]);
    }
    
    int left_max = 0;
    for (int i = 0; i < len; ++i) {
        left_max = std::max(data[i], left_max);
        if (left_max == right_min[i]) {
            vec.push_back(left_max);
        }
    }
    return vec;
}

int main() 
{
    int array[] = {1, 2, 4, 3, 9, 5, 6, 12, 15};
    int length = sizeof(array)/sizeof(int);

    func(array, length);

    return 0;
}
```