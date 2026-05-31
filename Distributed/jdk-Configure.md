<!--
 * @Author: JohnJeep
 * @Date: 2022-11-02 23:37:28
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-31 18:33:59
 * @Description: JDK environment configuration
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

JDK 环境配置

## Linux

1. 打开文件：`vim /etc/profile`，末尾添加

```sh
export JAVA_HOME=/usr/local/java/jdk1.8.0_171
export JRE_HOME=${JAVA_HOME}/jre
export CLASSPATH=.:${JAVA_HOME}/lib:${JRE_HOME}/lib
export PATH=${JAVA_HOME}/bin:$PATH		
```

2. 环境变量生效

   `source /etc/profile`

3. 添加软链接

   `ln -s /usr/local/java/jdk1.8.0_171/bin/java /usr/bin/java`

   > 注意正确的 jdk 安装路径。

4. 检查

   ```sh
   java --version 
   javac
   ```

   参考：https://www.cnblogs.com/stulzq/p/9286878.html





## Windows