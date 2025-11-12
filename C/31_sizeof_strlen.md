<!--
 * @Author: JohnJeep
 * @Date: 2020-08-13 21:14:28
 * @LastEditTime: 2025-11-12 22:42:26
 * @LastEditors: JohnJeep
 * @Description: sizeof与strlen区别
-->

在C++中，`sizeof`和`strlen`都是用于获取长度的操作，但它们的工作方式和应用场景有显著的不同。

1. **`sizeof` 运算符**：
   - `sizeof`是一个编译时一元运算符，用于计算其操作数所占用的内存大小（以字节为单位）。
   - 它可以用于任何数据类型，包括基本类型（如int、double）、结构体、类、数组等。
   - 当用于数组时，`sizeof`返回整个数组所占的字节数。例如，对于`int arr[10]`，`sizeof(arr)`将返回`10 * sizeof(int)`。
   - 当用于指针时，`sizeof`返回指针本身的大小（通常为4或8字节，取决于系统），而不是它所指向的内存块的大小。
   - 当用于字符串字面量时，`sizeof`会包括字符串末尾的空字符'\0'。
2. **`strlen` 函数**：
   - `strlen`是一个运行时函数，定义在`<cstring>`头文件中，用于计算一个以空字符('\0')结尾的字符串的长度，不包括空字符本身。
   - 它只能用于以空字符结尾的字符串（即C风格字符串）。
   - 它从给定的指针开始遍历内存，直到遇到空字符，然后返回遇到的字符数（不包括空字符）。

例子1：字符数组

```cpp
#include <iostream>
#include <cstring>

int main() {
    char str[] = "Hello";
    std::cout << "sizeof(str): " << sizeof(str) << std::endl; // 输出6，因为包括空字符，数组大小为6个字符（每个1字节）
    std::cout << "strlen(str): " << strlen(str) << std::endl; // 输出5，因为直到空字符前的字符数

    return 0;
}
```

例子2：指针指向字符串

```cpp
#include <iostream>
#include <cstring>

int main() {
    const char* str = "Hello";
    std::cout << "sizeof(str): " << sizeof(str) << std::endl; // 输出指针的大小，通常是4或8
    std::cout << "strlen(str): " << strlen(str) << std::endl; // 输出5

    return 0;
}
```

例子3：数组和指针的区别

```cpp
#include <iostream>
#include <cstring>

int main() {
    char str1[] = "Hello"; // 数组
    const char* str2 = "Hello"; // 指针

    std::cout << "sizeof(str1): " << sizeof(str1) << std::endl; // 6
    std::cout << "sizeof(str2): " << sizeof(str2) << std::endl; // 4或8
    std::cout << "strlen(str1): " << strlen(str1) << std::endl; // 5
    std::cout << "strlen(str2): " << strlen(str2) << std::endl; // 5

    return 0;
}
```

例子4：非字符串的数组

```cpp
#include <iostream>

int main() {
    int arr[] = {1, 2, 3, 4, 5};
    std::cout << "sizeof(arr): " << sizeof(arr) << std::endl; // 20（假设int为4字节，5个元素）
    // strlen(arr); // 错误！strlen只能用于以空字符结尾的字符串，这里使用会导致未定义行为。

    return 0;
}
```

**总结**

- **sizeof**：获取内存占用大小，编译时确定
- **strlen**：获取字符串长度，运行时计算
- **sizeof** 包含结尾的'\0'，**strlen** 不包含
- 对指针使用 **sizeof** 得到的是指针本身的大小，而不是指向内容的大小
