<!--
 * @Author: JohnJeep
 * @Date: 2025-04-10 23:41:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-10 23:52:09
 * @Description: cgo learning
 * Copyright (c) 2025 by John Jeep, All Rights Reserved. 
-->
# Type

Golang 类型与 C 类型映射表

| Type C                 | Call method    | Go type          | Bytes (byte) | Numerical range                          |
| ---------------------- | -------------- | ---------------- | ------------ | ---------------------------------------- |
| char                   | C.char         | byte             | 1            | -128~127                                 |
| signed char            | C.schar        | int8             | 1            | -128~127                                 |
| unsigned char          | C.uchar        | uint8            | 1            | 0~255                                    |
| short int              | C.short        | int16            | 2            | -32768~32767                             |
| short unsigned int     | C.ushort       | uint16           | 2            | 0~65535                                  |
| int                    | C.int          | int              | 4            | -2147483648~2147483647                   |
| unsigned int           | C.uint         | uint32           | 4            | 0~4294967295                             |
| long int               | C.long         | int32 or int64   | 4            | -2147483648~2147483647                   |
| long unsigned int      | C.ulong        | uint32 or uint64 | 4            | 0~4294967295                             |
| long long int          | C.longlong     | int64            | 8            | -9223372036854776001~9223372036854775999 |
| long long unsigned int | C.ulonglong    | uint64           | 8            | 0~18446744073709552000                   |
| float                  | C.float        | float32          | 4            | -3.4E-38~3.4E+38                         |
| double                 | C.double       | float64          | 8            | 1.7E-308~1.7E+308                        |
| wchar_t                | C.wchar_t      | wchar_t          | 2            | 0~65535                                  |
| void *                 | unsafe.Pointer |                  |              |                                          |


# References

CGO 编程: https://chai2010.cn/advanced-go-programming-book/ch2-cgo/ch2-02-basic.html