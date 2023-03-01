

## 怎么用

Rapidjson [官网](https://rapidjson.org/zh-cn/md_doc_tutorial_8zh-cn.html) 非常详细的介绍了如何去使用，自己就不再把官方的文档般到这里来了。只总结一些使用的心得。

### 解析



### 创建



## 注意事项

- RapidJSON 在类型转换时会检查数值的范围。
- 字符串字面量的优化
  - 只储存指针，不作复制
- 优化“短”字符串
  - 在 `Value` 内储存短字符串，无需额外分配。
  - 对 UTF-8 字符串来说，32 位架构下可存储最多 11 字符，64 位下 21 字符（x86-64 下 13 字符）。
- 可选地支持 `std::string`（定义 `RAPIDJSON_HAS_STDSTRING=1`）

- 最小化 DOM 的内存开销。

  对大部分 32／64 位机器而言，每个 JSON 值只占 16 或 20 字节（不包含字符串）。

- 支持快速的预设分配器。

  - 它是一个堆栈形式的分配器（顺序分配，不容许单独释放，适合解析过程之用）。
  - 使用者也可提供一个预分配的缓冲区。（有可能达至无需 CRT 分配就能解析多个 JSON）

- 部分 C++11 支持

  - 右值引用（rvalue reference）
  - `noexcept` 修饰符
  - 范围 for 循环

## 查询 Value

### 查询 Array

缺省情况下，`SizeType` 是 `unsigned` 的 typedef。在多数系统中，Array 最多能存储 $2^{32}-1$ 个元素。

用法与 `std::vector` 类似，同时也支持 C++11 的 for 循环迭代。

### 查询 Object

- `FindMember()`:  检查成员是否存在并返回它的 `Value`。

  用于查询 Object ，比普通的迭代遍历对象要高效。因为当 `operator[](const char*)` 找不到成员，它会断言失败。若我们不确定一个成员是否存在，便需要在调用 `operator[](const char*)` 前先调用 `HasMember()`，这会导致两次查找，降低了效率。

  ```c++
  // 普通查询
  static const char* kTypeNames[] = 
      { "Null", "False", "True", "Object", "Array", "String", "Number" };
   
  for (Value::ConstMemberIterator itr = document.MemberBegin();
      itr != document.MemberEnd(); ++itr) {
      printf("Type of member %s is %s\n",
          itr->name.GetString(), kTypeNames[itr->value.GetType()]);
  }
  
  // 采用FindMember()
  Value::ConstMemberIterator itr = document.FindMember("hello");
  if (itr != document.MemberEnd()) {
      printf("%s\n", itr->value.GetString());
  }
  ```

### 查询 String

- 支持处理包含 `\0` 的字符。

  根据 RFC 4627，JSON String 可包含 Unicode 字符 `U+0000`，在 JSON 中会表示为 `"\u0000"`。问题是，C/C++ 通常使用空字符结尾字符串（null-terminated string），这种字符串把 `‘\0’` 作为结束符号。为了符合 RFC 4627，RapidJSON 支持包含 `U+0000` 的 String。若要获取这种字符串的 长度，调用  `GetStringLength()` 函数即可。

- API
  - `GetString()`
  -  `GetStringLength()`







# Reference

- 官方 Github: https://github.com/Tencent/rapidjson/
- 英文文档: https://rapidjson.org/
- 中文文档: https://rapidjson.org/zh-cn/index.html
- Unicode 字符代码: https://www.rapidtables.org/zh-CN/code/text/unicode-characters.html
- Unicode 15.0 Character Code Charts: https://www.unicode.org/charts/

