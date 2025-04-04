/**
 * @file nullptr.cpp
 * @author JohnJeep
 * @brief  nullptr 是否可以赋值给 std::string
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>
#include <string>


// 错误原因解释
// 将 nullptr 赋值给 std::string 类型的变量会导致运行时错误，
// 是因为 std::string 类型的赋值运算符（operator=）不接受 nullptr。
// 在 C++ 中，nullptr 是一个特殊的空指针常量，它不能直接转换为其他类型，
// 除非其他类型的构造函数或者赋值运算符明确支持从 nullptr 到该类型的转换。

// 当你试图将 nullptr 赋值给 std::string 类型的变量时，编译器会尝试调用
//  std::string 类型的赋值运算符来执行赋值操作。但是，由于 nullptr 无法隐式转换为 
//  std::string 类型，因此编译器无法找到适合的赋值运算符，导致编译失败或者运行时错误。

int main(int argc, char const *argv[])
{
    std::string  str;
    char *p = nullptr;
    std::cout << &p << std::endl;    // --> ok
    // std::cout << *p << std::endl; // error: segmentation fault 

    // runtime error: segmentation fault
    // str = p;   
    // std::cout << str << std::endl;  

    // error: segmentation fault
    // const char *p2 = nullptr;
    // str = std::string(p2);
    // std::cout << str << std::endl;

    // constructor  ---> ok
    const char *p2 = "good";
    str = std::string(p2);
    std::cout << str << std::endl;

    // asign operation  ---> ok
    const char *p1 = "hello";
    str = p1;
    std::cout << str << std::endl;
    
    return 0;
}
