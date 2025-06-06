<!--
 * @Author: JohnJeep
 * @Date: 2019-08-29 21:45:27
 * @LastEditTime: 2025-04-04 19:47:01
 * @LastEditors: JohnJeep
 * @Description: 函数指针与指针函数的基础知识
--> 

# 1. 指针函数

## 1.1. 定义

指针函数是返回结果的类型为指针的一个函数。其本质是一个函数，与普通函数的区别是，指针函数的返回值是一个指针，函数返回的数据是一个地址。

## 1.2. 格式

函数的定义格式如下：
- `类型说明符  *函数名(参数表)`
- 格式：` int *fun(x, y)`

## 1.3. 代码实现

```c
#include "stdio.h"

int *complare(int *x, int *y);

int main()
{
    int a = 5;
    int b = 10;
    int *p;

    p = complare(&a, &b);
    printf("%d \n", *p);

    getchar();
    return 0;
}


int *complare(int *x, int *y)
{
    if(*x > *y)
    {
        printf("较大值: %x \n", x);
        return x;
    }
    else
    {
        printf("较小值的memory为: 0X=%x \n", y);
         return y;
    }
}
```


# 2. 函数指针

## 2.1. 什么是函数指针？

函数的名称是一个指针，表示一个函数的入口地址。将函数名赋给一个指针变量，然后通过这个指针变量对其函数进行操作。

## 2.2. 函数指针定义格式

- ` 类型标识符 (*指针变量名)()` 
- 格式：` int (*p)() `

## 2.3. 特点

- 函数指针 `本质` 是一个指针变量，该指针指向这个函数。
- 指针变量p，只能指向函数的入口地址，不能指向一般变量或数组。
- 一个函数指针变量可以先后指向不同的函数，但是函数的类型应该与函数指针的类型一样。


## 2.4. 使用步骤

1、先声明函数指针类型或变量  
  ```
  int (*pointerFunc)(int a, int b);
  pointerFunc ret;
  ```
2、将变量指向函数的首地址   `ret = func;`
3、使用变量调用函数       `ret(11, 22)`



## 2.5. 代码实现

```c
#include "stdio.h"

int add(int x, int y)
{
    return x + y;
}

int sub(int x, int y)
{
    return x -y;
}

int multiple(int x, int y, int z)
{
    return (x * y * z);
}

// 定义函数指针 
int (*func)(int, int);

// 函数指针做函数参数
int funcPointerParam(int (*funcPointer)(int, int, int))  
{
    int val = funcPointer(3, 4, 5);  // 直接调用定义的函数指针
    printf("multi val: %d\n", val);
    return val;
}

int main()
{
    func = add;   //func指向函数的地址
    printf("add 函数地址: 0x%p\n", &add);
    printf("add 指向的数据: %x\n", add);

    printf("func指针的地址: %x\n", &func);
    printf("func指向函数add的地址: %x\n", func);
    int a = func(1, 3);
    printf("carry add operation: %d\n", a);


    func = &sub;
    printf("sub address: %x\n", &sub);
    printf("func指针的地址: %x\n", &func);
    printf("func指向函数sub的地址: %x\n", func);
    int b = func(4, 1);
    printf("carry add operation: %d\n", b);

   // 调用函数指针做函数参数
   funcPointerParam(multiple);   // multiple 指向函数的入口地址

    return 0;
}
```

## 2.6. 常见的函数指针声明

1. 结构体函数指针
2. 用typedef来定义一个指针函数
3. 直接声明一个函数指针


## 2.7. 为什么使用函数指针：

- 可以将实现同一功能的多个模块统一起来标识，更容易后期维护，系统结构更加清晰。
- 便于分层设计、利于系统抽象、降低耦合度以及使接口与实现分开。函数指针约定了函数的返回值和函数的参数这套协议，留下了一套接口，以便兼容新增的代码。
- 实现面向对象编程中的多态性。
- 通过函数指针做函数的参数，可以实现回调函数。任务调用者和任务的实现者可以分开编写。
- 在程序运行的时候根据数据的具体状态来选择相应的处理方式


## 2.8. 其它指针声明

```cpp
int board[8][8];     // 声明一个内含int数组的数组
int ** ptr;          // 声明一个指向指针的指针， 被指向的指针指向int
int * risks[10];     // 声明一个内含10个元素的数组， 每个元素都是一个指向int的指针
int (* rusks)[10];   // 声明一个指向数组的指针， 该数组内含10个int类型的值
int * oof[3][4];     // 声明一个3×4 的二维数组， 每个元素都是指向int的指针
int (* uuf)[3][4];   // 声明一个指向3×4二维数组的指针， 该数组中内含int类型值
int (* uof[3])[4];   // 声明一个内含3个指针元素的数组， 其中每个指针都指向一个内含4个int类型元素的数组
```


## 2.9. <font color=red> []、()、* 优先级比较 </font>

- [] 、()属于基本运算符，优先级相同，结合方向为：从左到右
- `*` 属于单目运算符，比[]、()的优先级低
- 各类优先级：单目 > 算数运算 > 移位 > 关系 > 位逻辑 > 逻辑 > 条件 > 赋值 > 逗号


## 2.10. 函数指针数组

格式
```c
typedef int (*p)(void *);
p pArray[arraySize];
```

