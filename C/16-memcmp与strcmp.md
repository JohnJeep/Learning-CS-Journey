<!--
 * @Author: JohnJeep
 * @Date: 2020-09-12 09:47:29
 * @LastEditTime: 2020-08-13 21:08:18
 * @LastEditors: Please set LastEditors
 * @Description: memcmp()函数与strcmp()函数用法
 * @FilePath: /16-memcmp与strcmp.md
--> 

## 参考
- [百度百科strcmp参考](https://baike.baidu.com/item/strcmp/5495571?fr=aladdin)
- [百度百科memcmp参考](https://baike.baidu.com/item/memcmp/5494788?fr=aladdin)


## memcmp函数
- 函数原型：` int memcmp(const void *str1, const void *str2, size_t n)); `
- 参数
  - str1： 指向内存块的指针。
  - str2： 指向内存块的指针。
  - n： 要被比较的字节数
- 返回值
  - 如果返回值 < 0，则表示 str1 小于 str2。
  - 如果返回值 > 0，则表示 str1 大于 str2。
  - 如果返回值 = 0，则表示 str1 等于 str2
- 功能
  - 把存储区 str1 和存储区 str2 的前 n 个字节进行比较。该函数是按字节进行比较，该函数位于string.h。



## strcmp函数
- 函数原型 `intstrcmp(const char *s1,const char *s2)`
- 返回值（比较结果返回整数）
  - s1小于s2则返回负数
  - s1等于s2则返回零
  - s1大于s2则返回正数。
- 功能
  - 比较字符串s1和s2，对比是以 `字符` 为单位。
  - 两个字符串自左向右逐个字符相比（按ASCII值大小相比较），直到出现不同的字符或遇'\0'为止


### memcmp()与strcmp()区别
- `memcmp()` 能比较除字符串以外其它复杂的数据类型，但不能判断两个结构体是否一定相等。
- `strcmp()` 只能比较两个字符串的大小内容是否相等，以 `字符` 为单位进行比较，而 `memcmp()`是以内存的 `字节数` 进行比较的。