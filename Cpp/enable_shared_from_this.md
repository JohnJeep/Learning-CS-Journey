<!--
 * @Author: JohnJeep
 * @Date: 2021-04-28 21:24:43
 * @LastEditTime: 2025-04-04 19:23:58
 * @LastEditors: JohnJeep
 * @Description: In User Settings Edit
-->

# 1. 什么是 enable_shared_from_this?

C++11 开始时支持 `enable_shared_from_this`，它一个模板类，定义在头文件 `<memory>`，其原型为： 

```cpp
template< class T > class enable_shared_from_this;
```

- `std::enable_shared_from_this` 能让其一个对象（假设其名为 t ，且已被一个 std::shared_ptr 对象 pt 管理）安全地生成其他额外的 std::shared_ptr 实例（假设名为 pt0, pt2, ... ） ，它们与 pt 共享对象 t 的所有权。

- 若一个类 T 继承 std::enable_shared_from_this<T> ，则会为该类 T 提供成员函数： shared_from_this 。 当 T 类型对象 t 被一个为名为 pt 的 std::shared_ptr<T> 类对象管理时，调用 T::shared_from_this 成员函数，将会返回一个新的 std::shared_ptr<T> 对象，它与 pt 共享 t 的所有权。


# 2. 为什么要用 enable_shared_from_this？
- 需要在类对象的内部中获得一个指向当前对象的 shared_ptr 对象。
- 如果在一个程序中，对象内存的生命周期全部由智能指针来管理。在这种情况下，要在一个类的成员函数中，对外部返回this指针就成了一个很棘手的问题。


# 3. 什么时候用？
- 当一个类被 `share_ptr` 管理，且在类的成员函数里需要把当前类对象作为参数传给其他函数时，这时就需要传递一个指向自身的 `share_ptr`。
- 在当前类的内部需要将当前类的对象传递给其它的类用。


# 4. 如何安全地将 this 指针返回给调用者?
- 一般来说，我们不能直接将this指针返回。如果函数将 this 指针返回到外部某个变量保存，然后这个对象自身已经析构了，但外部变量并不知道，此时如果外部变量再使用这个指针，就会使得程序崩溃。


# 5. 标准库中的源码
```cpp
  template<typename _Tp>
    class enable_shared_from_this
    {
    protected:
      constexpr enable_shared_from_this() noexcept { }

      enable_shared_from_this(const enable_shared_from_this&) noexcept { }

      enable_shared_from_this&
      operator=(const enable_shared_from_this&) noexcept
      { return *this; }

      ~enable_shared_from_this() { }

    public:
      shared_ptr<_Tp>
      shared_from_this()
      { return shared_ptr<_Tp>(this->_M_weak_this); }

      shared_ptr<const _Tp>
      shared_from_this() const
      { return shared_ptr<const _Tp>(this->_M_weak_this); }

#if __cplusplus > 201401L || !defined(__STRICT_ANSI__) // c++11 or gnu++11
#define __cpp_lib_enable_shared_from_this 201602
      weak_ptr<_Tp>
      weak_from_this() noexcept
      { return this->_M_weak_this; }

      weak_ptr<const _Tp>
      weak_from_this() const noexcept
      { return this->_M_weak_this; }
#endif

    private:
      template<typename _Tp0>
	void
	_M_weak_assign(_Tp0* __p, const __shared_count<>& __n) const noexcept
	{ _M_weak_this._M_assign(__p, __n); }

      // Found by ADL when this is an associated class.
      friend const enable_shared_from_this*
      __enable_shared_from_this_base(const __shared_count<>&,
				     const enable_shared_from_this* __p)
      { return __p; }

      template<typename, _Lock_policy>
	friend class __shared_ptr;

      mutable weak_ptr<_Tp>  _M_weak_this;
    };
```
- enable_shared_from_this类中的成员函数
  - (constructor)：构造一个 enable_shared_from_this 对象，是一个受保护的成员函数，成员属性为 `protected`。
  - (destructor)：销毁一个 enable_shared_from_this 对象，是一个受保护的成员函数，成员属性为 `protected`。
  - operator=：返回到 this 的引用，是一个受保护成员函数，成员属性为 `protected`。
  - shared_from_this：返回共享 `*this` 指针所有权的 shared_ptr，是一个 `public` 属性的成员函数。
  - weak_from_this(C++16)：返回共享 `*this` 所指针有权的 weak_ptr，是一个`public` 属性的成员函数。

# 6. 具体的代码示例

```cpp
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
```

# 7. 使用注意事项
- enable_shared_from_this 的常见实现为：其内部保存着一个对 this 的弱引用（例如 std::weak_ptr )。 std::shared_ptr 的构造函数检测无歧义且可访问的 (C++16 起) enable_shared_from_this 基类，并且若内部存储的弱引用没有被以存在的 std::shared_ptr 占有，则 (C++17 起)赋值新建的 std::shared_ptr 为内部存储的弱引用。为另一个 std::shared_ptr 所管理的对象构造一个 std::shared_ptr ，将不会考虑内部存储的弱引用，从而将导致未定义行为(undefined behavior)。
  
- 只允许在先前已被std::shared_ptr 管理的对象上调用 shared_from_this 。否则调用行为未定义 (C++17 前)抛出 std::bad_weak_ptr 异常（通过 shared_ptr 从默认构造的 weak_this 的构造函数） (自C++17 起)。

- enable_shared_from_this 提供安全的替用方案，以替代 std::shared_ptr<T>(this) 这样的表达式（这种不安全的表达式可能会导致 this 被多个互不知晓的所有者析构）。


# 8. 参考
- [cpp reference解释其用法](https://zh.cppreference.com/w/cpp/memory/enable_shared_from_this) 
- [enable_shared_from_this用法分析](https://bbs.huaweicloud.com/blogs/136193)
- [enable_shared_from_this类的作用和实现](https://www.shuzhiduo.com/A/l0dyNmW9ze/)
- [C++11标准库的一个工具类enable_shared_from_this<T>的作用及原理分析](https://www.cnblogs.com/jo3yzhu/p/11358400.html)
