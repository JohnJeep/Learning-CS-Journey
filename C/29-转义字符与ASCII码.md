参考：
- [百度百科--ASCII](https://baike.baidu.com/item/ASCII/309296?fromtitle=ascii%E7%A0%81%E8%A1%A8&fromid=19660475&fr=aladdin)
- [C语言转义字符表和ASCII码表](https://www.cnblogs.com/kanhaoniao/p/11323166.html)



说明 | 转义字符 | 十六进制(hex) | 十进制(dec)
---|---|---|---
空字符(Null) | \0 |0x00 | 0
空格 |  | 0x20 | 32
字符 0 | 0 | 0x30 | 48
回车(CR: carriage return) | \r | 0x0D | 13
换行(LF: NL line feed, new line) | \n | 0x0A | 10
退格符(BS) | \b | 0x08 | 8


- 字符串结尾用空字符表示：`\0`
- 并不是所有的控制字符都可用相应的字母或者数字表示，不能表示的要用转义字符。任何一个转义字符都对应一个字符。







