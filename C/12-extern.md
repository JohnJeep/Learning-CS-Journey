<!--
 * @Author: JohnJeep
 * @Date: 2020-05-21 15:05:29
 * @LastEditTime: 2023-01-04 18:02:43
 * @LastEditors: JohnJeep JohnJeep1985@gail.com
 * @Description: static和extern关键字
-->

# 1. 术语(Terminology)
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

# 2. extern
C/C++中使用 `extern` 声明的变量或函数，它们的作用域是全局的，告诉编译器使用该关键字声明的变量可以在本模块或其他模块中使用。只是 `声明(declaration)` 了变量，但是并没有 `定义(definition)` 该变量，需要在具体使用的地方去定义该变量。
```c
// 在某个.h 文件中声明了变量
extern int a;

// 在某个具体的.c或.cpp文件中使用
int a = 100;
```

# 3. extern "C"
- `extern "C"` 的作用是为了能够正确在 C++ 代码中调用 C 语言代码。 加上 `extern "C" `后指示编译器按 C 编译器编译这部分代码。使用它的本质原因是：C++ 函数的重载；C++ 中函数重载是C++编译器通过编译后生成的代码不止有函数名，还会带上参数类型，编译后生成的代码，会改变函数的名称，而 C 编译器编译函数时不会带上函数的参数类型，编译后生成的代码，函数的名称不会改变。

- 在混合编程中，如果我们不进行任何处理，而相互调用的话，在链接会出现找不到符号链接的情况。

- `extern "C"` 是C++的特性，是一种链接约定，通过它可以实现兼容C与C++之间的相互调用，即对调用函数能够达成一致的意见，使用统一的命名规则，使得实现方提供的接口和调用方需要的接口经按照指定规则编译后，得到的都是一致的函数符号命名。


- Ｃ++与c互相调用问题
  
  C++中调用C函数：使用 `extern "C"` 关键字
  ```c++
  // a.c 头文件内容
  #ifndef A_H
  #define A_H
  
  int Test(int a, int b);
  
#endif
  
  // 在xxx.cpp文件文件中调用 a.h 文件中的 Test() 函数
  #include "a.h"
#include "stdio.h"
  
  extern "C"
  {
    int Test(int a, int b);
}
  
  int func(int a, int b) {
  printf("result=%d/n", Test(a, b));
  
    return 0;
}
  ```


  - C如何调用C++函数：需要使用 `预处理宏` 和 `extern "C"` 关键字
    ```c++
    #ifdef __cplusplus
    extern "C"
    {
    #endif
    ...
    ...
    #ifdef __cplusplus
    }
    #endif
    ```




