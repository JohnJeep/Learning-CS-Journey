<!--
 * @Author: JohnJeep
 * @Date: 2020-08-28 14:15:17
 * @LastEditTime: 2025-04-04 19:53:39
 * @LastEditors: JohnJeep
 * @Description: main()理解
 * 
-->

# main() 中参数传递

一般编写的程序在 Linux 环境下运行时，主函数一般写成这种形式： `int main(int argc, char* argv[])`，为何 `main()` 函数中传递的参数值要写成 `int argc, char* argv[]` 下面我们来仔细的分析。

- `argc` 表示数组中字符串的的数量
- `*argv[]` 表示一个指针数组，也等价于 `**argv`
- 当使用argv中的形参时，需要从 `argv[1]` 开始，`argv[0]` 是保存程序的名字，而非用户的输入。


#  main()返回值

- 在 C/C++ 中，将 main 函数写成 `int main() { return 0;}` 时，返回值是 `int类型`；若写成 `void main(){}` 时; 函数没有返回值，有些编译器器可能会报错。
- 函数的返回值如果等于 `0`，则代表程序正常退出，返回其它数字的含义则由系统决定，通常，返回非零代表程序异常退出。
