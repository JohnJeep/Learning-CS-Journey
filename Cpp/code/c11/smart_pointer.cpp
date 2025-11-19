/*
 * @Author: JohnJeep
 * @Date: 2025-11-12 22:55:03
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-19 10:21:24
 * @Description: shared_ptr 引用计数的几种情况
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
 */
#include <iostream>
#include <memory>

int main() {
    // 创建一个shared_ptr，引用计数为1
    std::shared_ptr<int> p1(new int(10));
    std::cout << "p1 引用计数: " << p1.use_count() << std::endl; // 输出1

    // 情况1：拷贝构造，引用计数增加
    std::shared_ptr<int> p2(p1);
    std::cout << "p1 引用计数: " << p1.use_count() << std::endl; // 输出2
    std::cout << "p2 引用计数: " << p2.use_count() << std::endl; // 输出2

    // 情况2：拷贝赋值，引用计数增加
    std::shared_ptr<int> p3;
    p3 = p1;
    std::cout << "p1 引用计数: " << p1.use_count() << std::endl; // 输出3

    // 情况3：按值传递函数参数，引用计数增加
    // 假设有一个函数，按值接收shared_ptr
    // 在函数调用时，引用计数会增加，函数返回后，因为形参销毁，引用计数会减少

    // 情况4：按值返回shared_ptr，在返回时引用计数增加（返回副本）
    // 但注意，如果使用移动语义（std::move）则不会增加，而是转移所有权。

    // 情况5：weak_ptr 转换为 shared_ptr
    std::weak_ptr<int> wp = p1;
    std::shared_ptr<int> p4;
    std::cout << "p4 引用计数: " << p4.use_count() << std::endl; // 输出 0
    p4 = wp.lock();
    std::cout << "p4 引用计数: " << p4.use_count() << std::endl; // 输出 4

    return 0;
}
