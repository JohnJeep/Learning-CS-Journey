<!--
 * @Author: JohnJeep
 * @Date: 2020-09-05 15:56:29
 * @LastEditTime: 2021-04-02 11:43:58
 * @LastEditors: Please set LastEditors
 * @Description: strncpy()与memcpy()函数用法
--> 
<!-- TOC -->

- [0.1. strncpy()](#01-strncpy)
- [0.2. memcpy()](#02-memcpy)
- [0.3. strcpy()与memcpy()区别](#03-strcpy与memcpy区别)
- [0.4. 参考](#04-参考)

<!-- /TOC -->

## 0.1. strncpy()
- 定义
`char *strncpy(char *dest, const char *src, size_t count); `
  - dest：目标字符数组；
  - src：源字符数组；
  - count：要复制的最大字符数
- 功能
  - 把src所指向的字符串中以src地址开始的前n个字节(不包括\0，\0得自己手动加在*dest被复制之后)复制到dest所指的数组中，并返回被复制后的dest
- 注意事项
  - src和dest所指内存区域不可以重叠，且dest必须有足够的空间来容纳src的字符长度 + `\0'`
  - strcpy只是复制字符串，但不限制复制的数量，很容易造成缓冲溢出。strncpy要安全一些。
  - memcpy能够选择一段字符输出，strcpy复制全部的字符串。
  - 执行完strncpy()后，会覆盖原先dest字符数组中的数据。


## 0.2. memcpy()
- 定义
` void *memcpy(void *dest, void *src, size_t count); `
  - 参数
     - dest: 指向用于存储复制内容的目标数组，类型强制转换为 `void*` 指针。
     - src: 指向要复制的数据源，类型强制转换为 `void*` 指针。
     - count: 要被复制的 `字节数`。
  - 返回值
    - 返回一个指向目标存储区dest的指针
    - 成功时返回为零，错误时，返回非零值。
- 功能
  - 从src内存地址拷贝 count 个字节到 dest内存中。
- 注意事项
  - 指针src和指针dest所指的内存区域不能重叠
  - src和dest都不一定是数组，任意的可读写的空间均可。
  - 两个不同的数组之间拷贝，用 `sizeof()` 得到 `字节数 n，不是传入数组的长度len`.
  - 执行完 `memcpy()` 后，会覆盖原先 `dest` 字符数组中的数据。


## 0.3. strcpy()与memcpy()区别
- 复制的内容不同。
   - strcpy只能复制字符串，而memcpy可以复制任意内容，例如字符数组、整型、结构体、类等。
- 复制的方法不同。
   - strcpy不需要指定长度，它遇到被复制字符的串结束符"\0"才结束，所以容易溢出。memcpy则是根据其第3个参数决定复制的长度。
- 用途不同。
   - 通常在复制字符串时用strcpy，而需要复制其他类型数据时则一般用memcpy
- `memcpy()`是内存到内存之间拷贝最快的，相比`strcpy()` 和 `memmove()`
  > `memmove()` 函数也是将 src 指向的内存中的 `count` 个字符拷贝到 dest 指向的内存区域中。若目标区域(dest)和源区域(src)有重叠的话，`memmove` 能够保证源串(src)在被覆盖之前将 `重叠区域` 的字节拷贝到目标区域中，但 **复制后源内容会被更改**。但是当目标区域与源区域没有重叠则和 `memcpy()`函数功能相同。


## 0.4. 参考
- [百度百科strncpy讲解](https://baike.baidu.com/item/strncpy/8491017?fr=aladdin)
- [百度百科memcpy讲解](https://baike.baidu.com/item/memcpy/659918?fr=aladdin)
