<!--
 * @Author: JohnJeep
 * @Date: 2019-09-06 09:49:29
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:15:17
 * @Description: Debugging Notes
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

F8：step over 单步 遇到断点后，程序停止运行，按 F8 单步运行。

F7：step into 进入 配合 F8 使用。单步调试 F8 时，如果某行调用其他模块的函数，在此行 F7，可以进入函数内部，如果是 F8
则不会进入函数内容，直接单步到下一行。

Alt+shift+F7：step into mycode, 个人理解 F8 和 F7 的综合。1、没遇到函数，和 F8
一样；2、遇到函数会自动进入函数内部，和 F8 时按 F7 类似的

shift+F8：跳出 调试过程中，F7 进入函数内后，shift+F8 跳出函数，会回到进入前调用函数的代码。不是函数地方 shift+F8 跳出

F9：resume program 按翻译是重启程序 ，实际是 下个断点，当打多个断点是，F9 会到下一个断点

-------------------
（1）Step into：遇见函数调用语句，进入函数内部

（2）Step over：遇见函数调用语句，但不进入函数内部，跳过该函数。调试时，如果不能确定这个函数是否有错，一般先跳过函数而
不进入。

（3）Step out：从当前的函数中跳出，程序流程执行函数调用语句的下一步。

