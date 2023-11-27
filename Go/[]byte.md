# [] byte 字节切片

日常工作中，`[]byte` 会经常遇到，但对这个知识点总是模模糊糊的，有些地方不是特别的清楚，因此把自己对 `[]byte` 的理解梳理出来，供大家分享。

在 Go 语言中，`[]byte` 是一个 **字节切片** 类型，用于表示二进制数据或字节序列。它常用于处理文件、网络传输和加密等场景。**用作变量、函数的参数、函数的返回值以及结构体、接口中都很常用。**

在这里要理解两个概念：一个是字节（byte），另一个是切片（Slice）。

- `byte`：就是 `[]byte` 右边的部分，表示元素类型是 `byte`，一个字节的数据，是一个无符号的 8 位整数类型。在 Go 语言中，`byte` 类型实际上是 `uint8` 的别名，所以 `byte` 的取值范围是 0 到 255。Go 标准库中的 `byte` 实现：

  ```go
  // uint8 is the set of all unsigned 8-bit integers.
  // Range: 0 through 255.
  type uint8 uint8

  // byte is an alias for uint8 and is equivalent to uint8 in all ways. It is
  // used, by convention, to distinguish byte values from 8-bit unsigned
  // integer values.
  type byte = uint8
  ```

- `[]`: 也就是切片，`[]byte` 左边的部分，是 Go 语言中的一种数据类型，可以理解一种动态的数组，只不过数组的长度不是固定的。

因此，`[]byte` 切片表示一系列字节数据的集合，其中每个元素都是一个字节（byte）。

## 以下是一些常见的 `[]byte` 的用法和操作：

1. 创建 `[]byte`
   - 使用字符串字面量初始化
   
  ```go
     data := []byte("hello")
     ```
   
     使用 `make` 函数创建指定长度的空切片
   
     ```go
     data := make([]byte, 10)`
     ```
   
2. 获取 `[]byte` 长度

   - 使用 `len` 函数获取切片的长度：`length := len(data)`。

3. 访问和修改 `[]byte` 元素
   - 通过索引访问和修改切片中的单个字节：`data[0]`。访问下标为 0 的字节切片中的数据。

   - 也可以使用切片语法获取子切片

     ```go
     // 语法
     subSlice := data[start:end]   // 其中 start 和 end 分别是起始和结束索引
     ```

4. 连接 `[]byte`

   使用 go 中内建的 `append()` 连接两个 `[]byte` 切片，返回的结果是一个新的切片类型

   ```go
   result := append(data1, data2...)
   ```

6. 字符串与 `[]byte` 之间的转换
   - 将字符串转换为 `[]byte`

     ```go
     str := "hello"
     data := []byte(str)
     ```

   - 将 `[]byte` 转换为字符串

     ```go
     data := []byte("hello")
     str := string(data)
     ```

7. 比较 `[]byte`

   - 使用 `bytes.Equal` 函数比较两个 `[]byte` 切片是否相等

     ```go
     equal := bytes.Equal(data1, data2)
     ```

8. 其他操作
   - 用 `os` 包中的函数读取文件内容到 `[]byte` 切片中

     ```go
     // os package 下的 ReadFile
     func ReadFile(name string) ([]byte, error) {
      	...
     }

     // 调用 ReadFile 函数
     filename := "./a.txt"
     data, err := os.ReadFile(filename)
     ```

   - 用 `encoding/base64` 包进行 `[]byte` 的 Base64 编码和解码。
   
     ```go
     package main
     
     import (
     	"encoding/base64"
     	"fmt"
     )
     
     func main() {
     	// 要编码的原始数据
     	originalData := []byte("Hello, World!")
     
     	// 进行Base64编码
     	encodedData := base64.StdEncoding.EncodeToString(originalData)
     	fmt.Println("Base64 编码结果:", encodedData)
     
     	// 进行Base64解码
     	decodedData, err := base64.StdEncoding.DecodeString(encodedData)
     	if err != nil {
     		fmt.Println("解码时发生错误:", err)
     		return
     	}
     	fmt.Println("Base64 解码结果:", string(decodedData))
     }
     ```

上面清楚了 `[]byte` 的基本用法后，下面我们更深层次的理解 `[]byte`。


## 一、[]byte 表示的数值范围是多少
在 Go 语言中，`[]byte` 切片表示的数值范围是 0 到 255。`byte` 类型实际上是 `uint8` 类型的别名，它是一个无符号的 8 位整数类型。

由于 `byte` 类型的取值范围是 0 到 255（或者用十六进制表示是 0x00 到 0xFF），因此 `[]byte` 切片中的每个元素都是一个在这个范围内的整数。

需要注意的是，虽然 `byte` 类型的底层表示是无符号的，但在进行运算时，它会被视为一个 8 位的无符号整数。这意味着在进行加法、减法或其他算术运算时，会按照无符号整数的规则进行计算。

## 二、编译器将 data := []byte("hello") 解释成什么
程序中写了一行下面的代码，将字符串转化为字节切片

```go
data := []byte("hello")
```

对这一行代码编译器会做哪些操作：
1. 创建一个长度为 5 的 `[]byte` 切片。
2. 将字符串 `"hello"` 按照 UTF-8 编码转换为字节序列。
3. 将字节序列的每个字节分别赋值给 `[]byte` 切片的对应位置。

这个过程可以分解为以下步骤：
1. 编译器根据 `"hello"` 字符串字面量创建一个不可变的字符串值。
2. 编译器检测到将这个字符串字面量转换为 `[]byte` 类型，并生成代码以执行转换。
3. 在运行时，转换代码将字符串按照 UTF-8 编码转换为字节序列。
4. 创建一个长度为 5 的 `[]byte` 切片。
5. 将字节序列的每个字节依次复制到切片的对应位置。
6. 最后，变量 `data` 将引用这个切片。

这样，`data` 就成为一个包含字符串 `"hello"` 的字节序列的切片，每个元素都是一个字节。


## 三、字符串 "hello" 按照 UTF-8 编码转换为字节序列后，转换后的字节序列是什么？
当字符串 `"hello"` 按照 UTF-8 编码转换为字节序列后，转换后的字节序列如下所示：

```go
[104 101 108 108 111]
```

这个字节序列表示字符串 `"hello"` 中每个字符的 UTF-8 编码。每个数字代表一个字节，对应于字符串中相应位置的字符的 UTF-8 编码。这些数字就是 ASCII 值。

**为什么是一个数字是一个字节？**
>  根据 UTF-8 编码规则，英文字母和常见符号通常使用一个字节表示。

解读转换后的字节序列：
- 第一个字节 `104` 对应字符'h' 的 UTF-8 编码。
- 第二个字节 `101` 对应字符'e' 的 UTF-8 编码。
- 第三个字节 `108` 对应字符'l' 的 UTF-8 编码。
- 第四个字节 `108` 对应字符'l' 的 UTF-8 编码。
- 第五个字节 `111` 对应字符'o' 的 UTF-8 编码。


## 四、data := []byte("hello") 在编译器和内存中存储的格式是什么
在编译器和内存中，`data := []byte("hello")` 所表示的数据格式如下：

1. 编译器处理：
   - 在编译阶段，编译器会将字符串字面量 `"hello"` 解析为一个不可变的字符串值。
   - 编译器会为 `data` 变量分配内存空间，并确定其类型为 `[]byte` 切片。
   - 编译器会生成代码以执行将字符串按照 UTF-8 编码转换为字节序列的操作。
2. 内存中的存储格式：
   - 在运行时，`data` 变量引用一个 `[]byte` 切片的实例。
   - 切片的实例包含两个字段：
     - 指向底层数组的指针：指向实际存储字节数据的数组。
     - 长度：切片中字节数据的数量。

实际的字节数据存储在一个连续的、可变长度的数组中。对于 `data := []byte("hello")`，`data` 切片的底层数组中存储的字节数据为：`[104 101 108 108 111]`，对应字符串 `"hello"` 的 UTF-8 编码序列。而 `data` 变量通过指针和长度字段引用该底层数组。


## References
- ASCII 值常用对照表：https://tool.oschina.net/commons?type=4