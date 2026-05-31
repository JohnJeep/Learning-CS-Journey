<!--
 * @Author: JohnJeep
 * @Date: 2020-05-30 18:16:48
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 19:56:59
 * @Description: base event concurrency
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

- 事件循环（An Event Loop）：等待事件发生，当它发生时，检查事件类型，然后做相应的处理（可能是I/O请求或者调度其它事件）。
- 采用 `select()` 和 `poll()` 进行事件的接受。
- `select()` 检查I/O描述符集合。
- 基于事件的系统中，遵循的原则：不允许阻塞调用。
- 采用事件编程的缺点：代码比较复杂、并且与现代操作系统的有些机制集成度较困难。

