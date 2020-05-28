```
 * @Author:JohnJeep
 * @Date: 2020-05-27 10:12:26
 * @LastEditTime: 2020-05-27 10:12:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
```
<!-- TOC -->

- [1. 基础概念](#1-基础概念)
  - [1.1. class without poniter members](#11-class-without-poniter-members)
    - [1.1.1. 命名空间（namespace）](#111-命名空间namespace)
    - [1.1.2. 头文件](#112-头文件)
    - [1.1.3. 构造函数](#113-构造函数)
    - [1.1.4. 重载（overloading）](#114-重载overloading)
    - [1.1.5. const](#115-const)
    - [1.1.6. 参数传递与返回值传递](#116-参数传递与返回值传递)
    - [1.1.7. friend（友元）](#117-friend友元)
    - [1.1.8. 操作运算符重载（operator overloading） this](#118-操作运算符重载operator-overloading-this)
  - [1.2. class with poniter members(class 类中带有指针成员)](#12-class-with-poniter-membersclass-类中带有指针成员)
    - [1.2.1. 析构函数（destructor）](#121-析构函数destructor)
    - [1.2.2. 拷贝构造（copy construction）](#122-拷贝构造copy-construction)
    - [1.2.3. 拷贝赋值（copy assignment operator）](#123-拷贝赋值copy-assignment-operator)
  - [1.3. 模板（template）](#13-模板template)
    - [1.3.1. class template](#131-class-template)
    - [1.3.2. function template](#132-function-template)
  - [1.4. 面向对象编程（OOP）](#14-面向对象编程oop)
    - [1.4.1. 复合（composition）](#141-复合composition)
    - [1.4.2. 委托（delegation）](#142-委托delegation)
    - [1.4.3. 继承（Inheritance）](#143-继承inheritance)

<!-- /TOC -->


# 1. 基础概念
编程----写出大家风范。


## 1.1. class without poniter members

### 1.1.1. 命名空间（namespace）
- 语法
```
// 使用标准库中封装的内容。标准库中定义的所有名字都在命名空间 std 中
using namespace std
{
  .....
}
```
- `::` 作用域运算符
  ```
  std::out 从标准库中输出读取的内容，即编译器从操作符左侧名字的作用域中去寻找右侧的名字。
  ```
- 一般有三种方式去实现命名空间
  - 全部打开标准库中的内容。`using namespace std;`
  - 只打开标准库的部分内容。
    ```
    using std::cout;

    int main()
    {
      cout << ...;
      std::cin << ...;
      
      return 0;
    }
    ```
  - 在使用的时候根据需要打开
    ```
    int main()
    {
      std::cout << ...;
      std::cin  << ...;
     
      return 0;
    }


### 1.1.2. 头文件
- 头文件采用防御式声明 
    ```
    #ifndef __TEST_H
    #define __TEST_H

    #endif
    ```
- 什么时候用 `inline`
  - 函数在class类里面定义 


### 1.1.3. 构造函数
- 为什么要用构造函数？
  - 被用来创建类  
- 构造函数的 `函数名称` 与 `类的名称` 一样
- 函数参数有默认值。
  - 创建的对象有默认值时，应传入创建的对象默认值；
  - 创建的对象没有默认值时，应传入的默认值为 `0`；
- **函数没有返回值**
- 构造函数是一个成员函数，有一个 `this pointer`
- 应在构造函数里面的第一行设置初始列（initialize list），即初始化。
  - 语法：`: re(r), im(i)` 将变量设置为：`re=r, im=i`
- 构造函数可以放在 `private` 中。在单例模式（singleton）中就采用这种用法。


### 1.1.4. 重载（overloading）
- 常常发生在构造函数中。
- 函数名称相同，函数参数不同


### 1.1.5. const
- 加 `const` 后，不会改变数据的内容，不加`const`，则会改变数据的内容，一般数据的内容定义在 `private`中。
- 在类中采用 `const`修饰函数，需要在类调用时必须加 `const`
  ```
  声明：
  double real() const
  {
      return re;
  }

  调用：
  const complex fx(1, 2);
  ```

### 1.1.6. 参数传递与返回值传递
- pass by value 和 pass by reference
  - 传值是将整个的数据传递给调用者
  - 传引用本质是传指针。
    - 采用一个 `&` 符号表示。
    - 希望调用者对传递的数据不能进行修改，在数据前加 `const` 限制。
- 参数传递时：在能使用传reference的前提下，一般优先使用 `传引用` 而尽量少使用传值，并不是必须的。`传引用` 的速度比 `传值` 速度快。
- 函数返回值传递时：在能使用传reference的前提下，一般优先使用 `传引用` 而尽量少使用传值，并不是必须的。 
  
- 什么情况下不能使用引用传递（reference）？
  - 当一个函数参数的变量为局部变量时，不能使用传引用。因为变量在函数结束时，变量就被销毁了，不存在，若再传递引用，调用者则不能得到值，会出错。


### 1.1.7. friend（友元）
  - 在一个类中使用 friend，当前类中未定义的函数可以去拿当前类中的数据。
  - 同一个 class 中的各个 object 互为友元。
    ```
    函数定义：
    int func(const complex& param)
    {
        return value;
    }

    声明对象:
    complex c1;
    complex c2;
    c2.func(c2);      // 采用友元方式实现
    ```

### 1.1.8. 操作运算符重载（operator overloading） this
- 操作符重载有两种实现的方式
  - 非成员函数方式，即全局（域）的方式。函数名称没有 `class` 名称
  - 成员函数方式。
    - 函数的参数有一个隐藏的 `this` pointer，可以在函数里面去调用。

- 成员函数
  - 谁调用，`this` 就指向谁 

- 类名后直接加括号，不创建一个对象，代表一个临时对象（temp object）。
    ```
    //声明类
    class complex
    {
        public:
            // 主要放置函数
            ........

        private:
            // 定义数据
            ........
    }

    // 调用临时对象
    complex();
    ```


## 1.2. class with poniter members(class 类中带有指针成员)

### 1.2.1. 析构函数（destructor）
  - 表示：在类名称之前加 `~`。例如：`string::~string()` 
  - 位置：在class类的之外去定义。
  - 作用：在类死亡之前的前一刻调用，用于清除类中的内容（比如：释放内存）


### 1.2.2. 拷贝构造（copy construction）
  - 浅拷贝：copy pointer
  - 深拷贝：copy content 


### 1.2.3. 拷贝赋值（copy assignment operator） 
  ```
  将s1拷贝赋值给s2，分3步：
  // 在拷贝之前需要进行自我赋值检测。即自己把值赋给自己，保证在执行第二步操作时，指针有指向的位置。
  if (this == str)
  {
    return *this
  }

  1、将s2自己清空
  2、分配 s2大小的内存空间存放s1
  3、执行拷贝动作
  ``` 

- 静态对象（static object）：在作用域（scope）结束后，生命周期还存在，即没有结束，一直到整个程序结束了，它的生命周期也就结束了。
  - 静态的函数没有 `this` pointer，只能去处理静态的数据
    ```
    如何去调用？ 
    1、使用 object 调用。Account a.state(10);
    2、通过 class name 来调用。Account::state(10);
    ```


- 全局对象（global object）：整个程序结束了，它的生命周期也就结束了。


- C++内存分配
  - windows 下内存显示总是 `16` 的倍数，不是 16 的倍数，则填充为最靠近 16 的倍数的大小。 
  - `new`  先分配空间，再调用构造函数
  - `delete` 先调用析构函数，再释放空间  
  - 注意：采用 `array new` 的方式创建一块内存空间，则一定要采用 ` array delete` 方式去释放内存，否则在涉及指针的可能会导致内存泄漏。泄漏的并不是整个分配的内存空间，而是分配的空间中数组没有被释放的部分。

构造一个函数
拷贝构造
操作符重载
拷贝赋值

## 1.3. 模板（template）
### 1.3.1. class template
- `template<typename T>`


### 1.3.2. function template
- `template<class T>`
- 编译器会做 `引数推导`

## 1.4. 面向对象编程（OOP）
### 1.4.1. 复合（composition）


### 1.4.2. 委托（delegation）
- 两个类之间通过指针相连


### 1.4.3. 继承（Inheritance）
- 三种继承：public、private、protect
- 内存角度探讨
  - 构造：由内而外。先调用base(父类)的默认构造函数，然后才执行自己。
  - 析造：由外而内。先执行自己，然后再调用 base(父类)的析构函数。

- 数据的继承：继承的是父类内存中的数据。
- 函数的继承：继承的是父类的调用权利。

- 继承与虚函数（virtual function）的结合
  - non-virtual: 不希望子类（derived）重写
  - virtual: 希望子类重写（override）父类，父类已有默认值
  - pure-virtual: 子类（derived）中必须重写(override)父类，父类没有默认值。

- 在使用框架结构去设计程序的时候，常常使用 `Template Method` 设计模式去实现。





作业
- string
- 实现复合、继承、委托三种方式

