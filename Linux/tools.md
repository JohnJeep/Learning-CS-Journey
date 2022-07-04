<!--
 * @Author: JohnJeep
 * @Date: 2021-03-22 19:04:41
 * @LastEditTime: 2022-01-27 15:42:34
 * @LastEditors: DESKTOP-0S33AUT
 * @Description: 积累Linux下常用的开发工具
-->

# 1. 通用

- DBeaver：一款基于Java 开发，免费开源的通用数据库管理和开发工具。DBeaver适用于所有开发人员、SQL程序员、数据库管理员和分析人员等，它支持任何具有JDBC驱动程序的数据库，EE版本还支持非JDBC数据源（MongoDB，Cassandra，Redis，DynamoDB等）。
- SysMonTask: 可视化的任务管理器。



# 2. 日志框架

- **spdlog：一个快速的 C++ 日志库，只包含头文件，兼容 C++11。**
- log4cxx：Java 社区著名的 Log4j 的 C++ 移植版，用于为 C++ 程序提供日志功能，以便开发者对目标程序进行调试和审计。
- log4cplus：一个简单易用的 C++ 日志记录 API，它提供了对日志管理和配置的线程安全、灵活和任意粒度控制（也基于 Log4j）。
- Log4cpp：一个 C++ 类库，可以灵活地记录到文件、syslog、IDSA 和其他目的地（也基于 Log4j）。
- google-glog：一个 C++ 语言的应用级日志记录框架，提供了 C++ 风格的流操作和各种辅助宏。
- Pantheios：一个类型安全、高效、泛型和可扩展性的 C++ 日志 API 库（号称 C++ 领域速度最快的日志库）。
- POCO：还提供了一个 好的日志支持文档。
- ACE：ACE 也有日志支持。
- Boost.Log：设计的非常模块化，并且可扩展。
- Easylogging++：轻量级高性能 C++ 日志库（只有一个头文件）。
- G3log：一个开源、支持跨平台的异步 C++ 日志框架，支持自定义日志格式。基于 g2log 构建，提升了性能，支持自定义格式。
- Plog：可移植、简单和可扩展的 C++ 日志库。


# 3. 内存泄漏检测工具

- valgrind
- mtrace
- AddressSanitizer(ASan) : 该工具为 gcc 自带，4.8以上版本都可以使用，支持Linux、OS、Android等多种平台，不止可以检测内存泄漏，它其实是一个内存错误检测具，可以检测的问题有：内存泄漏、堆栈和全局内存越界访问、free后继续使用、局部内存被外层使用、Initialization order bugs。
- VLD (Visual Leak Detector) 是一款开源检测内存泄露软件，Windows 下 visual studio 中非常好用的内存泄漏检测工具。

