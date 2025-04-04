<!--
 * @Author: JohnJeep
 * @Date: 2023-05-27 16:55:08
 * @LastEditors: JohnJeep
 * @LastEditTime: 2023-05-27 17:05:33
 * @Description: declaration 与 definition 区别
 * Copyright (c) 2023 by John Jeep, All Rights Reserved. 
-->


# 术语（Terminology）

- 何为声明（declaration）？

  告诉编译器某个东西的类型和名称，即不提供存储的位置和具体实现的细节。 

  ```c++
  extern itn x;                   // 变量声明
  std::size_t func(int num);      // 函数声明
  class Widget;                   // 类声明
  
  template<typename T>            // 模板声明
  class Student;
  ```


- 何为定义（definition）？

  编译器给变量分配内存空间，即提供存储的位置和具体实现的细节。

  ```c++
  int x;          // 对象定义，编译器为对象分配内存；此处对象非面向对象中的对象
  
  // 对function 或function template而言，定义提供了代码的本体
  std::size_t func(int num)
  {
    std::size_t a;
    return (a + 10);
  }
  	
  // class 定义
  class Widget
  {
  public:
    Widget();
    ~Widget();
  ......
  };
  
  // 模板定义
  template<typename T>            
  class Student
  {
  public:
    Student();    
    ~Student();    
  };
  ```

  注意：在程序中变量可以有`多次声明`，但只能 `定义一次`。


- 签名（signature）

  C++ 官方定义中，签名就是函数的参数部分，不包括函数的返回类型。但习惯上一般把函数返回类型和参数都作为签名的部分。

  例如函数声明中 `std::size_t func(int num);`，func 函数签名为 `std::size_t` 和 `int`