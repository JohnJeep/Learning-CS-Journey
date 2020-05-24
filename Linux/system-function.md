```
 * @Author: your name
 * @Date: 2020-05-23 23:12:17
 * @LastEditTime: 2020-05-23 23:12:17
 * @LastEditors: your name
 * @Description: 系统函数的使用
 ```
- 文件函数包括三部分内容
  - `file descriptor`  文件描述符
  - `file pointer(fp)` 文件指针
  - `file buffer`     文件数据缓冲区 


- 全部变量 `errno` 在Linux中存放位置 `/usr/include`
- 可以使用 `perror()` 打印错误的信息


- 文件的实际权限 = 用户给定的权限和本地的掩码取反做位与（&）操作






























