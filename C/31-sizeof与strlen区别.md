<!--
 * @Author: JohnJeep
 * @Date: 2020-08-13 21:14:28
 * @LastEditTime: 2020-08-17 16:06:01
 * @LastEditors: Please set LastEditors
 * @Description: sizeof与strlen区别
 * @FilePath: /31-sizeof与strlen区别.md
-->

`sizeof` 是求数据类型所占的空间大小，而 `strlen` 是求字符串的长度，字符串以 `\0` 结尾。

## sizeof()
- `sizeof()` 是运算符，其值在编译时就计算好了，参数可以是数组、指针、类型、对象、函数等。因此 `sizeof` 不能用来返回使用动态内存分配空间的大小。


## strlen()
- `strlen()` 是一个函数，要在运行时才能计算，参数必须是字符型指针 `char*`。当数组名作为参数传入时，实际上数组就退化成指针了。
- 它的功能是：返回字符串的长度。该字符串可能是自己定义的，也可能是内存中随机的，该函数实际完成的功能是从代表该字符串的第一个地址开始遍历，直到遇到结束符 `\0`，返回的长度大小不包括 `\0`。
