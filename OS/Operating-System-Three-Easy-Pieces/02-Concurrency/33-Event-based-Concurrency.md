<!--
 * @Author: JohnJeep
 * @Date: 2020-05-30 18:16:48
 * @LastEditTime: 2020-05-30 18:29:51
 * @LastEditors: Please set LastEditors
 * @Description: 基于事件的并发
--> 

- 事件循环（An Event Loop）：等待事件发生，当它发生时，检查事件类型，然后做相应的处理（可能是I/O请求或者调度其它事件）。
- 采用 `select()` 和 `poll()` 进行事件的接受。


`select()` 检查I/O描述符集合。


















