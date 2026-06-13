<!--
 * @Author: JohnJeep
 * @Date: 2020-08-11 22:43:12
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:30:29
 * @Description: fprintf in C language
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- 函数原型：`int fprintf (FILE* stream, const char*format, [argument])`
  - stream: 指向 FILE 对象的指针
  - format: 这是 C 字符串，包含了要被写入到流 stream 中的文本。format
    标签属性是`%[flags][width][.precision][length]specifier`
  - argment: 参数列表


- 函数功能
  - 根据指定的格式，向输出流(stream)写入数据(argment)
  - fprintf( )会根据参数 format 字符串来转换并格式化数据，然后将结果输出到参数 stream
    指定的文件中，直到出现字符串结束('\0')为止
