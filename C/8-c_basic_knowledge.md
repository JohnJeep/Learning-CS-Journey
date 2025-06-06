<!--
 * @Author: JohnJeep
 * @Date: 2019-08-29 21:45:27
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-04 19:45:30
 * @Description: basic knowledgement
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# 1. 理解面向对象和面向过程编程

- 面向对象
  - 本质是以建立模型体现出来的抽象思维过程和面向对象的方法；
  - 把构成问题事务分解成各个对象，建立对象的目的不是为了完成一个步骤，而是为了描叙某个事物在整个解决问题的步骤中的行为
- 面向过程
  - 一种以过程为中心的编程思想，分析出解决问题所需要的步骤，然后用函数把这些步骤一步一步实现，使用的时候一个一个依次调用就可以了


# 2. while

```C
while ( expression )
  statement;
```
- expression为真执行，为假则跳出
  - 所有非零的值都为真，只有0被视为假
- 单独的`;` 表示空语句，应该独占一行，可读性更好


# 3. 赋值语句 `=`

- 数据对象(Data Object):  用于储存值的数据存储区域
- 可修改的左值(modifiable Lvalue)：用于标识可修改的对象。或者叫对象定位值(object locator value)


# 4. do ... while

```C
do {
    statement;
} while (expresion);
```
- 先执行后判断，为真则执行，为假则跳出
- while() 和for()先判断后执行
- 一般而言， 当循环涉及初始化和更新变量时， 用for循环比较合适， 而在其他情况下用while循环更好


# 5. if...else

```C
if ( expression )
  statement1;
else {
    statement2;
}
```
- expression为真（非0）,则执行statement1；如果expression为假或0， 则执行else后面的statement2
- if和else之间只允许有一条语句（简单语句或复合语句）


# 6. continue

- 用作占位符：在使用while循环时，后边语句部分采用` ; `可以从continue代替
- 让程序跳过循环体的余下部分


# 7. 文件流

- C语言处理的是流(stream),不直接处理文件。流（stream） 是一个实际输入或输出映射的理想化数据流。 
- windows操作系统可以使用内嵌的` Ctrl+Z `字符来标记文件结尾。而Linux中的文件结尾结尾标志为：`Ctrl+D`
- 用` getchar()`读取文件检测到文件结尾时将返回一个特殊的值， 即EOF（end of file）,在C语言中定义为` EOF=-1 `
- ` scanf() `函数检测到文件结尾时也返回EOF。


# 8. 动态内存分配

- calloc函数
  - `void *calloc(size_t m,size_tsize);`m个大小为size字节的对象分配存储空间，把已分配的空间所有位都初始化为0
- malloc函数：
  - `void *malloc(size_t,size)` 大小为size字节的对象分配存储空间，存储空间的初始值不确定


# 9. 条件编译(precompilation)
- #if
```C
#if 表达式
    程序段1
[# else
    程序段2]
#enif
```

- #ifndef
```C
#ifndef 标识符
    程序段1
[# else
    程序段2]
#enif
```

- 为什么要用预编译？
> 不同的源码文件，可能会引用同一个头文件（比如stdio.h）。编译的时候，头文件必须一起编译。为了节省时间，编译器会在编译源码之前，先编译头文件。这保证了头文件只需编译一次，不必每次用到的时候，都重新编译了。
<font color=red>用来声明宏定义的`#define`命令，就不会被预编译 </font>


# 10. 双引号与单引号区别

- 单引号` ' ` :  代表一个整数，数值对应于该字符在ASCII字符集中的序列值；一个字符就是一个字节。
- 双引号` " `: 表示一个字符串，进行指针运算，代表字符指针；
其中一个字符也是一个字符串，大小为两个字节(后面为\0)。
- <font color=red> 注意: </font> 
  - C编译器接受字符和字符串的比较，无任何意义
  - C编译器允许字符串对字符变量赋值，只能得到错误

# 11. 其它知识点 

b = ++a 先对a加一后赋值为b

b = a++ 先赋值后a加一

% 求余数运算符结果的符号与 % 左边的操作符号相同
  > 例如：-45 % 8 结果为-5

大写字母与小写字母ASCII码相差32（小写字母大）

指针运算通常只有减法没有加法，差值是两个指针变量之间相差的元素个数


# 12. References

- [C语言](https://blog.csdn.net/cb673335723/article/details/78205191)