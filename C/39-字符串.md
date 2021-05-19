<!--
 * @Author: JohnJeep
 * @Date: 2021-05-19 21:58:05
 * @LastEditTime: 2021-05-19 22:05:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
# 1. 字符串
- 字符串是一种特殊的 char 类型的数组，指向 char 类型数组的指针。
- 字符串与 char 数组的区别在于长度，字符会自动在尾部加上一个长度 `\0`，而 char 型数组的长度就是其字符的个数。


- `sizeof()` 
  > 遍历字符串，遇到 `\0` 就终止，返回的结果是第一个`\0`前字符元素的个数。`指针声明` 的字符串不能使用 `sizeof()` 方式求字符串的长度。

- `strlen()` 
  > 求字符串变量占用内存空间的大小，可以用来求字符串的长度；返回的是存储字符串的变量所占用的内存空间大小。