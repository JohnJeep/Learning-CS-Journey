# 1. IOStream(输入输出流)

输入流 `cin`

- `getline()` 终端输入缓冲区中时可以输入 `空格`。
- `ignore()` 忽略缓冲区指定的数据
- `peek()` 读缓冲区的数据，若有数据，则读出缓冲区的一个数据；没有数据，则读出无数据。
- `putback()`

输出流 `cout`

- `flush()` 刷新缓冲区的数据
- `put()` 将字符一个一个地输出到标准输出上
- `write()`
- `width()`
- `fill()`  

文件IO流

- `ofstream`建立一个输出流对象，将数据输出到指定文件中。
- `ifstream` 建立一个输入流对象，将从文件中读到的数据输出到终端上。

# 2. Meata Programming(元编程)

元编程针对**类型**进行操作。而一般的编程是对 **变量或对象** 进行操作。

`tuple` 标准库的底层是采用递归的方式去实现的。 

三个核心 API 接口

1. `td::make_tuple`: 构造元组
2. `std::get`: 获得元组某个位置的值
3. `std::tie`: 元组拆包

```cpp
#include <tuple>
#include <iostream>
auto get_student(int id)
{
// 返回类型被推断为 std::tuple<double, char, std::string>
if (id == 0)
    return std::make_tuple(3.8, 'A', "张三");
if (id == 1)
    return std::make_tuple(2.9, 'C', "李四");
if (id == 2)
    return std::make_tuple(1.7, 'D', "王五");
    return std::make_tuple(0.0, 'D', "null");
    // 如果只写 0 会出现推断错误, 编译失败
}
int main()
{
    auto student = get_student(0);
    std::cout << "ID: 0, "
    << "GPA: " << std::get<0>(student) << ", "
    << "成绩: " << std::get<1>(student) << ", "
    << "姓名: " << std::get<2>(student) << '\n';
    double gpa;
    char grade;
    std::string name;
    // 元组进行拆包
    std::tie(gpa, grade, name) = get_student(1);
    std::cout << "ID: 1, "
    << "GPA: " << gpa << ", "
    << "成绩: " << grade << ", "
    << "姓名: " << name << '\n';
}
```

# 3. Resource acquisition is initialization(RAII)

Resource acquisition is initialization 简写为 RAII，翻译为：资源获取即初始化。RAII要求，资源的有效期与持有资源的对象的生命期严格绑定，即由对象的构造函数完成资源的分配(获取)，同时由析构函数完成资源的释放。在这种要求下，只要对象能正确地析构，就不会出现资源泄露问题。