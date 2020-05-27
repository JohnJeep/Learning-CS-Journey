```
 * @Author:JohnJeep
 * @Date: 2020-05-27 10:12:26
 * @LastEditTime: 2020-05-27 10:12:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
```
<!-- TOC -->

- [1. class](#1-class)
  - [1.1. class without poniter members](#11-class-without-poniter-members)
    - [1.1.1. 头文件](#111-头文件)
    - [1.1.2. 构造函数](#112-构造函数)
    - [1.1.3. 重载（overloading）](#113-重载overloading)
    - [1.1.4. const](#114-const)
    - [1.1.5. 参数传递与返回值传递](#115-参数传递与返回值传递)
    - [1.1.6. friend（友元）](#116-friend友元)
    - [1.1.7. 操作运算符重载（operator overloading） this](#117-操作运算符重载operator-overloading-this)
  - [1.2. class with poniter members(class 类中带有指针成员)](#12-class-with-poniter-membersclass-类中带有指针成员)

<!-- /TOC -->


# 1. class

## 1.1. class without poniter members
### 1.1.1. 头文件
- 头文件采用防御式声明 
    ```
    #ifndef __TEST_H
    #define __TEST_H

    #endif
    ```

- 什么时候用 `inline`
  - 函数在class类里面定义 


### 1.1.2. 构造函数
- 为什么要用构造函数？
  - 被用来创建类  
- 构造函数的 `函数名称` 与 `类的名称` 一样
- 函数参数有默认值。
  - 创建的对象有默认值时，应传入创建的对象默认值；
  - 创建的对象没有默认值时，应传入的默认值为 `0`；
- **函数没有返回值**
- 应在构造函数里面的第一行设置初始列（initialize list），即初始化。
  - 语法：`: re(r), im(i)` 将变量设置为：`re=r, im=i`
- 构造函数可以放在 `private` 中。在单例模式（singleton）中就采用这种用法。


### 1.1.3. 重载（overloading）
- 常常发生在构造函数中。
- 函数名称相同，函数参数不同


### 1.1.4. const
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

### 1.1.5. 参数传递与返回值传递
- pass by value 和 pass by reference
  - 传值是将整个的数据传递给调用者
  - 传引用本质是传指针。
    - 采用一个 `&` 符号表示。
    - 希望调用者对传递的数据不能进行修改，在数据前加 `const` 限制。
- 参数传递时：在能使用传reference的前提下，一般优先使用 `传引用` 而尽量少使用传值，并不是必须的。`传引用` 的速度比 `传值` 速度快。
- 函数返回值传递时：在能使用传reference的前提下，一般优先使用 `传引用` 而尽量少使用传值，并不是必须的。 
  
- 什么情况下不能使用引用传递（reference）？
  - 当一个函数参数的变量为局部变量时，不能使用传引用。因为变量在函数结束时，变量就被销毁了，不存在，若再传递引用，调用者则不能得到值，会出错。


### 1.1.6. friend（友元）
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

### 1.1.7. 操作运算符重载（operator overloading） this
- 操作符重载有两种实现的方式
  - 非成员函数方式，即全局（域）的方式。函数名称没有 `class` 名称
  - 成员函数方式。  


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



内联函数

ostream&














