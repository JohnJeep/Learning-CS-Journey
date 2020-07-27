<!--
 * @Author: JohnJeep
 * @Date: 2020-09-04 11:42:29
 * @LastEditTime: 2020-07-27 15:52:33
 * @LastEditors: Please set LastEditors
 * @Description: memset()函数用法
 * @FilePath: /14-memset.md
--> 

## 参考
- [memset百度百科](https://baike.baidu.com/item/memset/4747579?fr=aladdin#reference-[1]-982208-wrap)


## 函数
- 定义 `void *memset(void *s, int ch, size_t n);`
  - 函数解释：将s中当前位置后面的n个字节 （typedef unsigned int size_t ）用 ch 替换并返回 s 
  - 该函数只能取ch的后八位赋值给你所输入的范围的每个字节，无论ch多大只有后八位二进制有效
  - ch的范围为：0~255
  - 对字符数组操作时则取后八位赋值给字符数组，其八位值作为ASCII码。
- 函数原型：`extern void *memset(void *buffer, int c, int count)` 
  - buffer：为指针或是数组,
  - c：是赋给buffer的值
  - count： 要填充的 `字节数`。
- 作用
  - 为新申请的内存按字节进行初始化。
  - 对较大的结构体或数组进行清零，这种方法最快。
  - 不能用它将int数组初始化为0和-1之外的其他值（除非该值高字   节和低字节相同）。

<font color=red>注意点</font>
- 内存操作都是按照字节为单位进行处理，即 1字节
- 填充的数 count 按照字节为单位设置。
- 定义数组为int类型时，传入的数为`sizeof(int)`的整数倍，而不是数组的长度。
