<!--
 * @Author: JohnJeep
 * @Date: 2019-09-06 9:18:29
 * @LastEditTime: 2021-01-20 23:18:27
 * @LastEditors: Please set LastEditors
 * @Description: typedef基础用法
--> 

<!-- TOC -->

- [1. Typedef Kyewords](#1-typedef-kyewords)
  - [1.1. 为什么使用typedef](#11-为什么使用typedef)
  - [1.2. 基本语法](#12-基本语法)
  - [1.3. typedef与数组](#13-typedef与数组)
  - [1.4. typedef与函数指针](#14-typedef与函数指针)
  - [1.5. 与#define比较](#15-与define比较)
  - [1.6. 参考](#16-参考)

<!-- /TOC -->


# 1. Typedef Kyewords
## 1.1. 为什么使用typedef
- 使用 typedef来编写更美观和可读的代码。所谓美观，意指 typedef 能隐藏笨拙的语法构造以及平台相关的数据类型，从而增强可移植性和以及未来的可维护性。 
- 给变量取一个好记且意义明确的新名字，
- 简化一些比较复杂的类型声明。


## 1.2. 基本语法 
- typedef
  - typedef：为现有类型取一个新的名称
  - 语法规则：`typedef  现有数据类型名称 新类型名称`
  - `typedef a b;`  给原类型a取一个别名为b


## 1.3. typedef与数组
- 定义相同类型和大小的数组
```C
typedef char arr[50];
arr text, data;  // 声明了一个text数组和一个data数组
```


## 1.4. typedef与函数指针
```C
typedef void (*PrintHelloHandle)(int); 

PrintHelloHandle pFunc;  // 声明一个函数指针为 pFunc 的别名
pFunc = printHello;      // 初始化函数指针，将 printHello 的函数的地址赋值给函数指针  pFunc

(*pFunc)(110);           // 调用函数指针，两种方式：(*pFunc)(110) 或 pFunc(110)
 
 //在其它地方的程序需要声明类似的函数指针，只需要简单使用
 PrintHelloHandle pFuncOther; // 声明一个函数指针为 pFuncOther 的别名
```

- 为什么要用函数指针与typedef结合
  - 在多个地方声明同一个类型的函数指针变量，简化代码


## 1.5. 与#define比较
- #define只是简单的字符串替换而typedef则是为一个类型起新名字
- 通常讲，typedef要比#define要好，特别是在有指针的场合。

## 1.6. 参考
- [百度百科typedef用法](https://baike.baidu.com/item/typedef/9558154?fr=aladdin)

