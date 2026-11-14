<!--
 * @Author: JohnJeep
 * @Date: 2026-11-14 06:22:40
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-11-14 10:48:14
 * @Description: sed usage
 * Copyright (c) 2026 by John Jeep, All Rights Reserved.
-->

sed 俗称做流编辑器，以行单位进行字符处理。

选项参数
- `-n`：默认情况下，脚本执行完后，自动打模式空间中的内容，使用 `-n` 可以屏蔽自动打印。
- `-i`: 将修改后的源文件保存
- `-E`：使用扩展正则表达式
- `-e`: 使用多个脚本语句替换 时，需要用 `-e` 选项。


脚本语句参数
- `a(append)`：追加
- `i(insert)`：插入
- `d(delete)`：删除
- `s(substitution)`

```bash
# 将 for_tmp.sh 脚本文件中的所有 echo 替换为 printf，使用 -i 参数并将修改的内容保存，而非仅仅输出显示到标准输出上。
sed -i 's/echo/printf/g' for_tmp.sh
```
