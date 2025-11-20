<!--
 * @Author: JohnJeep
 * @Date: 2021-04-28 21:24:43
 * @LastEditTime: 2025-11-20 11:50:20
 * @LastEditors: JohnJeep
 * @Description: In User Settings Edit
-->

- [1. 什么是 enable\_shared\_from\_this?](#1-什么是-enable_shared_from_this)
- [2. 为什么要用 enable\_shared\_from\_this？](#2-为什么要用-enable_shared_from_this)
  - [2.1. 问题场景](#21-问题场景)
  - [2.2. 解决方案：enable\_shared\_from\_this](#22-解决方案enable_shared_from_this)
- [3. 什么时候用？](#3-什么时候用)
  - [3.1. 主要使用场景](#31-主要使用场景)
    - [3.1.1. 异步回调](#311-异步回调)
    - [3.1.2. 容器中存储自身指针](#312-容器中存储自身指针)
    - [3.1.3. 链式调用](#313-链式调用)
  - [3.2. 重要注意事项](#32-重要注意事项)
    - [3.2.1. 必须在已有 shared\_ptr 的情况下使用](#321-必须在已有-shared_ptr-的情况下使用)
    - [3.2.2. 构造函数中不能使用](#322-构造函数中不能使用)
    - [3.2.3. 继承要求](#323-继承要求)
- [4. 标准库中源码](#4-标准库中源码)
- [5. 代码示例](#5-代码示例)
- [6. References](#6-references)


# 1. 什么是 enable_shared_from_this?

C++11 开始时支持 `enable_shared_from_this`，它一个模板类，定义在头文件 `<memory>`，其原型为： 

```cpp
template< class T > class enable_shared_from_this;
```

`std::enable_shared_from_this` 能让其一个对象（假设其名为 t ，且已被一个 std::shared_ptr 对象 pt 管理）安全地生成其他额外的 std::shared_ptr 实例（假设名为 pt0, pt2, ... ） ，它们与 pt 共享对象 t 的所有权。

若一个类 T 继承 std::enable_shared_from_this<T> ，则会为该类 T 提供成员函数： shared_from_this 。 当 T 类型对象 t 被一个为名为 pt 的 std::shared_ptr<T> 类对象管理时，调用 T::shared_from_this 成员函数，将会返回一个新的 std::shared_ptr<T> 对象，它与 pt 共享 t 的所有权。

# 2. 为什么要用 enable_shared_from_this？

用于解决一个特定的问题：**当对象需要获取指向自身的 `shared_ptr` 时，如何安全地创建而不导致重复计数或悬空指针**。

## 2.1. 问题场景

考虑以下代码：

```cpp
class Widget {
public:
    void process() {
        // 错误！这会创建新的控制块，导致重复计数
        auto self_ptr = std::shared_ptr<Widget>(this);
        // 使用 self_ptr...
    }
};

int main() {
    auto widget = std::make_shared<Widget>();
    widget->process();  // 会导致未定义行为
}
```

这里的问题是：`widget` 已经有一个 `shared_ptr` 在管理它，但在 `process()` 中又用 `this` 创建了另一个 `shared_ptr`，这会创建新的控制块，导致对象被多次删除。

## 2.2. 解决方案：enable_shared_from_this

```cpp
class Widget : public std::enable_shared_from_this<Widget> {
public:
    void process() {
        // 正确！共享现有的控制块
        auto self_ptr = shared_from_this();
        // 安全地使用 self_ptr...
        
        // 也可以获取 weak_ptr
        auto weak_self = weak_from_this();
    }
    
    std::shared_ptr<Widget> get_shared() {
        return shared_from_this();
    }
};

int main() {
    auto widget = std::make_shared<Widget>();
    widget->process();  // 安全，共享同一个控制块
    
    auto another_ptr = widget->get_shared();  // 也安全
}
```

`enable_shared_from_this` 的主要用途是：

- **安全地在对象内部获取指向自身的 `shared_ptr`**
- **避免创建多个控制块导致的重复计数问题**
- **在异步操作、回调函数等场景中保持对象生命周期**

总结：使用`enable_shared_from_this`是为了在类的内部安全地获取指向当前对象的`shared_ptr`，避免创建多个控制块，从而防止未定义行为。

# 3. 什么时候用？

使用`enable_shared_from_this`的主要场景是：在类的成员函数中，需要获取指向当前对象的`shared_ptr`。如果直接使用`this`指针来创建另一个`shared_ptr`，则会创建一个新的控制块，这与原有的`shared_ptr`的控制块是分离的，会导致同一个对象被多个控制块管理，这是错误的。

## 3.1. 主要使用场景

### 3.1.1. 异步回调

```cpp
class Connection : public std::enable_shared_from_this<Connection> {
public:
    void start_async_operation() {
        auto self = shared_from_this();
        
        // 在回调中捕获 shared_ptr，确保对象存活
        async_operation([self]() {
            self->handle_completion();
        });
    }
    
private:
    void handle_completion() {
        // 处理完成
    }
};
```



### 3.1.2. 容器中存储自身指针

```cpp
class Node : public std::enable_shared_from_this<Node> {
    std::vector<std::shared_ptr<Node>> children;
    
public:
    void add_child() {
        auto child = std::make_shared<Node>();
        children.push_back(child);
        
        // 父节点也需要在子节点中存储
        child->parent = shared_from_this();  // 假设 parent 是 weak_ptr<Node>
    }
};
```



### 3.1.3. 链式调用

```cpp
class Builder : public std::enable_shared_from_this<Builder> {
public:
    std::shared_ptr<Builder> set_name(const std::string& name) {
        this->name = name;
        return shared_from_this();
    }
    
    std::shared_ptr<Builder> set_value(int value) {
        this->value = value;
        return shared_from_this();
    }
};
```



## 3.2. 重要注意事项

### 3.2.1. 必须在已有 shared_ptr 的情况下使用

不能将`enable_shared_from_this`用于栈上分配的对象，因为栈上对象没有被`shared_ptr`管理。

```cpp
class Widget : public std::enable_shared_from_this<Widget> {};

// 错误用法：
Widget w;
// w.shared_from_this();  // 未定义行为！没有现有的 shared_ptr

// 正确用法：
auto widget = std::make_shared<Widget>();
widget->shared_from_this();  // 安全
```



### 3.2.2. 构造函数中不能使用

```cpp
class Widget : public std::enable_shared_from_this<Widget> {
public:
    Widget() {
        // shared_from_this();  // 错误！在构造函数中不能使用
    }
};
```



### 3.2.3. 继承要求

```cpp
// 正确：公有继承
class Derived : public std::enable_shared_from_this<Derived> {};

// 如果需要多态，使用 CRTP 的变体
class Base : public std::enable_shared_from_this<Base> {
public:
    virtual ~Base() = default;
};

class Derived : public Base {
public:
    std::shared_ptr<Derived> shared_from_this() {
        return std::static_pointer_cast<Derived>(
            Base::shared_from_this());
    }
};
```



enable_shared_from_this 的常见实现为：其内部保存着一个对 this 的弱引用（例如 std::weak_ptr )。 std::shared_ptr 的构造函数检测无歧义且可访问的 (C++16 起) enable_shared_from_this 基类，并且若内部存储的弱引用没有被以存在的 std::shared_ptr 占有，则 (C++17 起)赋值新建的 std::shared_ptr 为内部存储的弱引用。为另一个 std::shared_ptr 所管理的对象构造一个 std::shared_ptr ，将不会考虑内部存储的弱引用，从而将导致未定义行为(undefined behavior)。

只允许在先前已被std::shared_ptr 管理的对象上调用 shared_from_this 。否则调用行为未定义 (C++17 前)抛出 std::bad_weak_ptr 异常（通过 shared_ptr 从默认构造的 weak_this 的构造函数） (自C++17 起)。

enable_shared_from_this 提供安全的替用方案，以替代 std::shared_ptr<T>(this) 这样的表达式（这种不安全的表达式可能会导致 this 被多个互不知晓的所有者析构）。

# 4. 标准库中源码

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
enable_shared_from_this 类中的成员函数
- (constructor)：构造一个 enable_shared_from_this 对象，是一个受保护的成员函数，成员属性为 `protected`。
- (destructor)：销毁一个 enable_shared_from_this 对象，是一个受保护的成员函数，成员属性为 `protected`。
- operator=：返回到 this 的引用，是一个受保护成员函数，成员属性为 `protected`。
- shared_from_this：返回共享 `*this` 指针所有权的 shared_ptr，是一个 `public` 属性的成员函数。
- weak_from_this(C++16)：返回共享 `*this` 所指针有权的 weak_ptr，是一个`public` 属性的成员函数。

# 5. 代码示例

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



# 6. References

- [cpp reference解释其用法](https://zh.cppreference.com/w/cpp/memory/enable_shared_from_this) 
- [enable_shared_from_this用法分析](https://bbs.huaweicloud.com/blogs/136193)
