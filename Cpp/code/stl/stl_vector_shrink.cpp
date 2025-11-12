/*
 * @Author: JohnJeep
 * @Date: 2025-11-08 10:33:09
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-12 22:52:06
 * @Description: vector 容量收缩与扩展测试
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
 */
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>

using namespace std;

void shrink_expand_test()
{
    std::vector<int> v;
    cout << "No assign value, size: " << v.size() << ", capacity: " << v.capacity() << endl;
    // v.reserve(100);
    for (int i = 0; i < 5; ++i) {
        v.push_back(i);
    }
    cout << "size: " << v.size() << ", capacity: " << v.capacity() << endl;

    v.shrink_to_fit();
    cout << "After shrink_to_fit(), size: " << v.size() << ", capacity: " << v.capacity() << endl;
    
    v.clear(); // 与 v.resize(0) 等价
    cout << "After clear(), size: " << v.size() << ", capacity: " << v.capacity() << endl;
}


void resize_clear_test()
{
    std::vector<int> v = {1, 2, 3, 4, 5};
    cout << "Initial size: " << v.size() << ", capacity: " << v.capacity() << endl;
    v.resize(8); // 扩展到 8 个元素，新增元素默认初始化为 0, capacity 增加
    cout << "After resize(3), size: " << v.size() << ", capacity: " << v.capacity() << endl;

    cout << "element: ";
    for (const auto& val : v) {
        cout << val << ", ";
    }

    v.clear(); // 清空元素，但不改变容量
    cout << "\nAfter clear(), size: " << v.size() << ", capacity: " << v.capacity() << endl;
}

void performance_test() {
    std::vector<int> vec;
    auto start = std::chrono::high_resolution_clock::now();
    // vec.reserve(1000000);
    for (int i = 0; i < 1000000; ++i) {
        auto op_start = std::chrono::high_resolution_clock::now();
        vec.push_back(i);
        auto op_end = std::chrono::high_resolution_clock::now();
        
        // 某些 push_back 操作会明显更慢（重新分配时）
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            op_end - op_start);
        if (duration.count() > 1000) { // 检测慢操作
            std::cout << "慢操作在 i=" << i 
                      << ", 耗时: " << duration.count() << "ns" << std::endl;
        }
    }
}

int main(int argc, char const *argv[])
{
    shrink_expand_test();
    resize_clear_test();
    performance_test();
    
    return 0;
}

