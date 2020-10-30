<!--
 * @Author: JohnJeep
 * @Date: 2020-10-30 09:38:07
 * @LastEditTime: 2020-10-30 16:18:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
- 解释采用哪种shell运行脚本: `!#/bin/sh`
- 多条语句之间使用 `;` 分割开
- 输入的命令采用 `()` 括起来，shell会fork一个新的字shell进程执行括号里面的内容。
  
## 基本语法
- 数据类型：只有数据类型，且为字符串
- 变量
  - 一般自定的名称使用大写。 
  - 环境变量: 将本地变量提升为环境变量，export VARNAME=value
  - 本地变量: VARNAME=value
  - `unset VARNAME`: 删除已定义的环境变量或本地变量
  - `env | grep VARNAME`: 查看系统中的环境变量 VARNAME
  - 命令代换: `$(commmand)` 执行括号里面的命令功能
  - 算术代换: 用于基本的算术计算。
    - `echo $[11+12]` 两个数相加
    - `echo $[2#10+16#12]` 二进制的 10 加上十六进制的 12，结果按照十进制显示
- 分支语句  
  - if语句，格式
  ```
    echo "Is it morning? Please answer yes or no."
    read YES_OR_NO
    if [ "$YES_OR_NO" = "yes" ]; then          # 取值必须使用""
        echo "Good moning!"
    elif [ "$YES_OR_NO" = "no" ]; then
        echo "Good afternoon!"
    else
        echo "Sorry, $YES_OR_NO not recognized. Enter yes or no."
    fi
  ``` 
- 函数
- 命令代换


