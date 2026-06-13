<!--
 * @Author: JohnJeep
 * @Date: 2019-08-30 13:36:34
 * @LastEditTime: 2026-05-31 18:19:30
 * @LastEditors: JohnJeep
 * @Description: restrict keyword notes
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
-->

C99 新增一个限定符：`restrict` 

作用：只用于限制指针。告诉编译器，想要修改当前指针指向内存的的数据，只能通过当前指针操作，不能通过除当前指针以外的变量
或指针进行操作。
