```
 * @Author: JohnJeep
 * @Date: 2020-05-23 23:12:17
 * @LastEditTime: 2020-05-23 23:12:17
 * @LastEditors: JohnJeep
 * @Description: 系统函数的使用
 ```
- 文件函数包括三部分内容
  - `file descriptor`  文件描述符
  - `file pointer(fp)` 文件指针
  - `file buffer`     文件数据缓冲区 


- 全部变量 `errno` 在Linux中存放位置 `/usr/include`
- 可以使用 `perror()` 打印错误的信息


- 文件的实际权限 = 用户给定的权限和本地的掩码取反做位与（&）操作

- `open` 函数
  - 创建：`O_CREAT` 或采用 `截断为0的方式创建 O_TRUNC`
  - 读写：`O_RDWR`
  - 只读：`O_RDONLY`
  - 只写：`O_WRONLY`
  - 文件是否存在：`O_EXCL`
  

- `read/write` 函数
  - `-1` 读/写文件失败
  - `0`  文件读完了或文件写成完了
  - `>0` 读/写文件的字节数


- `lseek` 函数 
  - 获取文件的长度
  - 移动文件指针
  - 文件拓展（只能向文件的中间或尾部扩展，不能向前端扩展）


























