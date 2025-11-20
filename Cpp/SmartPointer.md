<!--
 * @Author: JohnJeep
 * @Date: 2020-05-27 10:12:26
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-11-20 11:36:28
 * @Description: 智能指针用法
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->

- [1. smart pointer(智能指针)](#1-smart-pointer智能指针)
  - [1.1. 为什么要用](#11-为什么要用)
  - [1.2. 原理](#12-原理)
  - [1.3. 优点](#13-优点)
- [2. auto\_ptr](#2-auto_ptr)
- [3. unique\_ptr](#3-unique_ptr)
  - [3.1. 为什么要用 unique\_ptr](#31-为什么要用-unique_ptr)
  - [3.2. 初始化](#32-初始化)
  - [3.3. 成员函数](#33-成员函数)
  - [3.4. deleter](#34-deleter)
  - [3.5. 注意点](#35-注意点)
- [4. shared\_ptr](#4-shared_ptr)
  - [4.1. 为什么要用 shared\_ptr](#41-为什么要用-shared_ptr)
  - [4.2. 成员函数](#42-成员函数)
  - [4.3. 底层原理](#43-底层原理)
  - [4.4. 初始化](#44-初始化)
  - [4.5. 用法](#45-用法)
  - [4.6. deleter](#46-deleter)
  - [4.7. 注意点](#47-注意点)
- [5. weak\_ptr](#5-weak_ptr)
  - [5.1. 为什么要用 weak\_ptr](#51-为什么要用-weak_ptr)
  - [5.2. 初始化](#52-初始化)
  - [5.3. 底层原理](#53-底层原理)
  - [5.4. 成员函数](#54-成员函数)
  - [5.5. 注意点](#55-注意点)


# 1. smart pointer(智能指针)

自 C++11 起 C++ 标准库提供了两种类型的智能指针：`shared_ptr` 和 `unique_ptr`。而所有的智能指针都被封装在标准库的 `<memory>` 头文件中，要使用智能指针必须引入 `#include <memory>` 头文件。

## 1.1. 为什么要用

1. 动态分配内存时可能会出现一些问题
  - 忘记释放内存，会造成内存泄漏
  - 有指针引用内存的情况下，释放了内存，产生引用非法内存的指针。
2. 需要更加安全的来管理动态内存。动态内存分配常用 `new` 和 `delete` 来分配内存。不使用 smart pointer 时，用动态内存分配时，可能会忘记 delete，导致内存泄漏；也可以使用异常捕获，但是会导致代码比较臃肿，不易阅读和维护。因此智能指针可以很好的解决这个问题。
3. 负责自动释放所指向对象的内存资源。智能指针就是一个类（class），当智能对象超出了类的作用域时，类会自动调用析构函数，释放资源。

## 1.2. 原理

智能指针底层源码采用类模板（class template）来实现的，并不是一个简单的普通指针。可以用下面的模型来简单的表示

<img src="./figures/smart-pointer.png">

## 1.3. 优点

在函数结束时自动释放内存空间，不需要手动释放内存空间。

# 2. auto_ptr

- auto_ptr 智能指针采用所有权模式。
- 已被 C++11弃用，潜在内存崩溃问题。
- 存在非法的申请内存时，在编译期时可能通过，但程序在运行时可能会出错。

# 3. unique_ptr

`unique_ptr` 实现的是一种独一无二拥有权 (exclusive ownership) 的概念。保证一个对象和其相应资源同一时间内只能被一个智能指针拥有(ownership)。当 unique_ptr 被销毁时，它所指向的对象也就自动销毁。

## 3.1. 为什么要用 unique_ptr

它对于避免资源泄露，例如以 `new` 创建对象后因为发生异常而忘记调用 `delete`特别有用。 

## 3.2. 初始化

unique_ptr 智能指针提供三种方式进行对象的初始化。构造函数中初始化、移动构造函数中初始化 `std::move()`、采用 `reset()` 成员函数进行初始化。

- **unique_ptr 不允许执行 copy(拷贝) 和 assignment(赋值) 操作**。但是可以用 `std::move()` 语义将对象的拥有权转移。

  ```cpp
  // initialize a unique_ptr with a new object
  std::unique_ptr<ClassA> up1(new ClassA);
  
  // copy the unique_ptr
  std::unique_ptr<ClassA> up2(up1);  // ERROR
  
  // assign the unique_ptr, transfer ownership from up1 to up3
  std::unique_ptr<ClassA> up3(std::move(up1));    // OK
  ```

- 当程序试图将一个 `unique_ptr` 赋值给另一个时，如果源 `unique_ptr` 是个临时右值，编译器允许这么做；如果源 `unique_ptr` 将存在一段时间，编译器将禁止这么做。

  ```CPP
    unique_ptr<string> pu1(new string ("hello world")); 
    unique_ptr<string> pu2; 
    pu2 = pu1;                                      // 不允许拷贝构造
    unique_ptr<string> pu3; 
    pu3 = unique_ptr<string>(new string ("You"));   // 允许
  ```

- 想要执行 ` pu2 = pu1;` 的操作，又要保证指针的安全。可以用C++有一个标准库函数 `std::move()`，让你能够将一个 `unique_ptr`赋给另一个。

  尽管转移所有权后 还是有可能出现原有指针调用（调用就崩溃）的情况。但是这个语法能强调你是在 `转移所有权`，让你清晰的知道自己在做什么，从而`不乱调用原有指针`。

- `unique_ptr` 可以转移对象的拥有权。使`unique_ptr` 不必一定拥有对象，它也可以是 empty；例如：当它被默认构造函数创建时。

  ```cpp
  std::unique_ptr<std::string> ip;
  ip = nullptr;
  ip.reset();
  ```

## 3.3. 成员函数

- `move()`: 转移对象的拥有权
- `reset()`: 销毁内部对象并接受新对象的所有权并将该智能指针被置为空，等价于 up = nullptr
- `release()`: 放弃内部对象的拥有权
- `swap()`: 交换两个指针指向的对象(即交换所拥有的对象)。
- `get()`: 获得内部对象的指针

## 3.4. deleter

unique_ptr 也有自己的 删除器。

```cpp
// 但lambda 表达式中没有写捕获参数时，要实现自己的删除器，需要在模板参数中指定其参数类型
using func = void(*)(Stu*);    // void 类型的函数指针
unique_ptr<Stu, func> s1(new Stu(100), [](Stu* p){    
  delete p;
});


// 有捕获参数时，unique_ptr 模板参数类型为 仿函数的返回类型
unique_ptr<Stu, std::function<void (Stu*)>> s2(new Stu(200), [&](Stu* p){    
  delete p;
});

// 申请的内存为数组类型时，模板参数为数组类型
unique_ptr<Stu[]> ptr1(new Stu[3]);
```

独占的智能指针能管理数组类型的地址，能够自动释放。

```cpp
unique_ptr<Stu[]> ptr1(new Stu[3]);
```

C++11 中 shared_ptr 不支持下面的语法，自C++11之后的版本，开始支持下面的语法。

```cpp
shared_ptr<Stu[]> ptr1(new Stu[3]); 
```

## 3.5. 注意点

`unique_ptr` 智能指针创建对象时，在 C++11 版本没有提供 `std::make_unique()` 的方式去创建对象，只能用 `new` 关键字创建对象。 

```cpp
std::unique<Employee> employee(new Employee);

// 不需要显示去调用delete，智能智能自动去调用delete
```

`std::make_unique` 在 C++14 中引入，可用下面的方式去创建对象。

```cpp
auto employee = std:make_unique<Employee>();
```

若编译器版本只支持 C++11，可以自己封装一个 `std::make_unique` 函数去实现。

```cpp
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique(Args&& ...args) {
  return std::unique_ptr<T>(new T( std::forward<Args>(args)... ));
}
```

# 4. shared_ptr

`share_ptr` 实现的是一种共享所有权 (shared ownership)的概念。多个智能指针可以指向同一个对象，该对象和它的相关资源会在最后一个指针的指向 (`reference`) 被销毁时，得到释放。

## 4.1. 为什么要用 shared_ptr

shared_ptr 是为了解决 auto_ptr 在对象所有权上的局限性(auto_ptr 是独占的)，在使用引用计数的机制上提供了可以共享所有权的智能指针。  

## 4.2. 成员函数

- `use_count()` 返回引用计数的个数
- `unique()` 返回指针对象的拥有者是否唯一(等价于 use_count=1)
- `swap()` 交换两个 shared_ptr 对象(即交换所拥有的对象)
- `reset()` 放弃内部对象的所有权或拥有对象的变更, 会引起原有对象的引用计数的减少。简单来说，主要有两个作用：1、让指针指向另一块内存；2、重置指针，即让引用计数变为 0。
- `get()` 获得被 `shared_ptr` 包裹的内部对象, 即获得原始的指针，类似 `*p` 这样的。
- `get_deleter()` 返回删除器 (deleter) 的地址。

## 4.3. 底层原理

采用 `引用计数` 的方法，记录当前内存资源被多少个智能指针引用，引用计数的内存在堆上分配。

当新增一个指针时，`引用计数` 加1，当释放时 `引用计数` 减一。只有引用计数为0时，智能指针才会自动释放引用的内存资源。

**调用 constructor，copy constructor，copy asignment 函数都会导致引用计数增加。**

## 4.4. 初始化

`shared_ptr` 有四种初始化方式。

1. 通过构造函数（constructor）初始化。

2. 通过移动构造函数（move constructor）或者拷贝构造函数（copy constructor）初始化。

3. 通过 `reset()` 函数进行初始化。

4. 通过 `make_shared` 初始化。用 `shared_ptr` 进行初始化时，不能将一个普通指针直接赋值给智能指针，因为一个是指针，一个是类。但可以通过 `make_shared` 函数或者通过构造函数传入普通指针，并可以通过 `get()` 函数获得普通指针。 

  ```cpp
  shared_ptr<string> p = new string("hello");               // ERROR
  shared_ptr<string> p(new string("hello"));                // OK
  shared_ptr<string> p = make_shared<string>("hello"));     // OK
  ```

## 4.5. 用法

直接使用智能指针对象。

```cpp
shared_ptr<Stu> st5 = make_shared<Stu>(9527);
st6->setValue(777);
st6->getValue();
```

获取智能指针对象的原始指针，通过 `get()` 方法。

```cpp
shared_ptr<Stu> st1(new Stu(007));
Stu* p = st1.get();
p->setValue(100);
p->getValue();
```

## 4.6. deleter

shared_ptr 默认的删除器函数(deleter)不能自动析构申请的是数组类型对象的内存，因此需要手动实现一个删除器.当申请的内存不是数组类型时，不需要手动实现删除器。

```cpp
shared_ptr<Stu> st(new Stu(10), [](Stu* p){
    delete p;
});
```

其中对象中的第二个参数是匿名对象，这个匿名对象可以对象的外部实现后再传入进来，也可以在对象中使用 lambda 表达式，例如：`[](Stu* p){}`。

```cpp
// 定义一个自己的删除器：deleter,可以选择自己不手动实现
shared_ptr<string> str(new string("Implement my deleter"),
                       [](string* p) {
                           cout << "deleter: " << *p << endl;
                           delete p;
                       });
str = nullptr;

// 必须要手动实现删除器
// shared_ptr<Stu> St7(new Stu[5]);   // 执行 5 次 构造函数，析构函数执行一次，造成内存泄漏
shared_ptr<Stu> St7(new Stu[5], [](Stu* t){  // 改进版
    delete []t;
});
```

在删除数组内存时，除了自己编写删除器，也可以使用 C++ 提供的 std::default_delete<T>() 函数作为删除器，这个函数内部的删除功能也是通过调用 delete 来实现的，要释放什么类型的内存就将模板类型 T 指定为什么类型即可。具体处理代码如下：

```cpp
shared_ptr<Stu> st8(new Stu(5), default_delete<Stu>());
```

自己封装一个 `make_shared_array` 方法来让 `shared_ptr` 支持数组。

```cpp
template<typename T>
shared_ptr<T> make_shared_array(size_t len)
{
    return shared_ptr<T>(new T[len], default_delete<T[]>());
}

void test05()
{
    shared_ptr<Stu> t = make_shared_array<Stu>(4);
    cout << t.use_count() << endl;
}
```

完整的代码可在仓库中查看：[shared_ptr实现](./code/c11/shared_ptr.cpp)

## 4.7. 注意点

1. `shared_ptr` 可能导致内存泄漏。两个对象相互使用一个 `shared_ptr` 成员变量指向对方，会造成循环引用，从而导致内存泄漏。
2. 不能使用一个原始地址值初始化多个 shared_ptr。
3. 函数不能返回管理了 this 指针的 shared_ptr 对象。
4. shared_ptr 只提供 `operator*` 和 `operator->`，没有提供 `operator[]` 和指针运算。

# 5. weak_ptr

`weak_ptr` 是弱引用指针，是一种不控制对象生命周期的智能指针，指向一个 `shared_ptr` 管理的对象。`weak_ptr` 只提供一种访问手段，它不共享指针，不能操作资源。

## 5.1. 为什么要用 weak_ptr

1. 配合 `shared_ptr` 智能指针来进行工作，解决 `shared_ptr` 智能指针相互引用时死锁的问题。当两个 `shared_ptr`智能指针相互引用时，这两个指针的引用数永远不可能减到 0 ，导致资源永远不会释放。

2. 它是对 对象的一种弱引用，不会增加对象的 **引用计数**。

3. `weak_ptr ` 与 `shared_ptr`之间可以相互转化。`shared_ptr` 可以直接赋值给 `weak_ptr`；而 `weak_ptr `通过调用 `lock()` 函数来获得 `shared_ptr`。

   ```cpp
   std::shared_ptr<int> p1(new int(10));
   std::weak_ptr<int> wp = p1; // shared_ptr直接赋值给weak_ptr
   
   // weak_ptr通过调用lock()获得 shared_ptr
   std::shared_ptr<int> sp = wp.lock(); 
   ```

## 5.2. 初始化

`weak_ptr` 提供三种初始化的方式

1. 构造函数中初始化。
2. 拷贝构造函数初始化。
3. 通过隐式类型转换，shared_ptr 对象直接赋值给 weak_ptr 对象来初始化。

```cpp
shared_ptr<int> st(new int()); // Create a object

weak_ptr<int> wt1;
cout << "wt1.use_count = " << wt1.use_count() << endl;

weak_ptr<int> wt2(st);
cout << "wt2.use_count = " << wt2.use_count() << endl;

weak_ptr<int> wt3(wt1);
cout << "wt3.use_count = " << wt3.use_count() << endl;

weak_ptr<int> wt4 = st; // 通过隐式类型转换，shared_ptr对象直接赋值给 weak_ptr 对象
cout << "wt4.use_count = " << wt4.use_count() << endl;
```

## 5.3. 底层原理

`weak_ptr` 底层主要依赖于 `counter` 计数器类和 `shared_ptr` 赋值、构造等手段实现的。

- `counter` 对象的目地就是用来申请一个块内存来存引用基数。
- `share_ptr` 给出的函数接口为：构造，拷贝构造，赋值，解引用。

## 5.4. 成员函数

- `expired()` 检测所管理的对象是否已经释放, 如果已经释放, 返回 true; 否则返回 false。
- `lock()` 获取所管理的对象的强引用 `shared_ptr`；如果 `expired` 为 true, 返回一个空的 `shared_ptr`; 否则返回一个 `shared_ptr`, 其内部对象指向与 `weak_ptr` 相同。
- `reset()` 放弃被拥有物的拥有权，重新初始化为一个空的 `weak_ptr`。
- `use_count()` 返回所监测的 `shared_ptr` 共享对象的引用计数。

代码用例实现在工程库中：[Weak_ptr 智能指针用法](./code/c11/weak_ptr.cpp)

## 5.5. 注意点

- `weak_ptr` 没有重载 `*` 和`-> ` 但可以使用 `lock` 获得一个可用的 `shared_ptr` 对象。`weak_ptr` 在使用前需要检查合法性。
- `weak_ptr` 支持 `拷贝或赋值`, 但不会影响对应的 `shared_ptr` 内部对象的计数。
