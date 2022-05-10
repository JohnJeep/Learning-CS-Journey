/*
 * @Author: JohnJeep
 * @Date: 2021-05-07 19:12:43
 * @LastEditTime: 2022-05-08 16:03:29
 * @Description: 标准库中的 Array 容器用法
 */
#include <iostream>
#include <array>

void print(const std::array<int, 5>& arr)
{
    for (const auto& value : arr) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}

// 封装通用的迭代器
template<typename Iterator> 
void print_it(Iterator begin, Iterator end) 
{
    for (Iterator it = begin; it != end; ++it) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
}


int main(int argc, char *argv[])
{
    std::array<int, 5> data = {5, 3, 1, 6, 8};
    std::cout << "array: ";
    print(data);

    std::cout << "general iterator: ";
    print_it(data.cbegin(), data.cend());
    
    return 0;
}