<!--
 * @Author: JohnJeep
 * @Date: 2019-08-30 13:36:34
 * @LastEditTime: 2021-04-02 13:50:54
 * @LastEditors: Please set LastEditors
 * @Description: C语言中restrict关键字用法
--> 
- C99新增一个限定符：`restrict` 
- 作用
  - 只用于限制指针。告诉编译器，想要修改当前指针指向内存的的数据，只能通过当前指针操作，不能通过除当前指针以外的变量或指针进行操作。

