<!--
 * @Author: JohnJeep
 * @Date: 2021-08-08 01:18:00
 * @LastEditTime: 2021-08-08 01:45:40
 * @LastEditors: Windows10
 * @Description: 多线程并发
-->
# async
async() 提供一个接口，让一段功能或一个可调用对象在后台运行，成为一个独立的线程。


# future
Class future<> 允许你等待线程结束并获取其结果。

std::shared_future<>，允许你在多个地点等待和处理线程结束。



Data race (数据竞争): 两个线程并发操作同一块数据导致不可预期的行为。



Waiting and Polling (等待和轮训)