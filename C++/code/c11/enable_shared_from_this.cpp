/*
 * @Author: JohnJeep
 * @Date: 2021-01-10 18:08:34
 * @LastEditTime: 2021-04-05 17:17:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <stdlib.h>
#include <memory>
using namespace std;

// 比较推荐的写法
struct Good : std::enable_shared_from_this<Good> // note: public inheritance
{
    std::shared_ptr<Good> getptr() {
        return shared_from_this();
    }
};

// 错误的用法：用不安全的表达式试图获得 this 的 shared_ptr 对象
struct Bad
{
    std::shared_ptr<Bad> getptr() {
        return std::shared_ptr<Bad>(this);
    }
    ~Bad() { std::cout << "Bad::~Bad() called\n"; }
};
 
int main()
{
    // 正确的用法: 两个 shared_ptr 共享同一个对象
    std::shared_ptr<Good> gp1 = std::make_shared<Good>();
    std::shared_ptr<Good> gp2 = gp1->getptr();
    std::cout << "gp2.use_count() = " << gp2.use_count() << '\n';
 
    // 错误的用法: 调用 shared_from_this 但其没有被 std::shared_ptr 占有 
    try {
        Good not_so_good;
        std::shared_ptr<Good> gp1 = not_so_good.getptr();
    } 
    catch(std::bad_weak_ptr& e) {
        // 在 C++17 之前，编译器不能捕获 enable_shared_from_this 抛出的std::bad_weak_ptr 异常
        // 这是在C++17之后才有的特性
        std::cout << e.what() << '\n';    
    }
 
    // 错误的用法，每个 shared_ptr 都认为自己是对象的唯一拥有者
    // 调用错误的用法，会导致两次析构 Bad的对象，第二次析构时，指针指向的空间已经被析构，
    // 会导致程序出错
    std::shared_ptr<Bad> bp1 = std::make_shared<Bad>();
    std::shared_ptr<Bad> bp2 = bp1->getptr();
    std::cout << "bp2.use_count() = " << bp2.use_count() << '\n';

    return 0;
}  