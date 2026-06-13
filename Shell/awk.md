<!--
 * @Author: JohnJeep
 * @Date: 2026-05-19 20:58:43
 * @LastEditors: JohnJeep
 * @LastEditTime: 2026-05-19 21:01:02
 * @Description: awk usage in engineering practice
 * Copyright (c) 2026 by John Jeep, All Rights Reserved. 
-->

# awk 工程实战指南

`awk` 是文本处理的三剑客之一（grep/sed/awk），擅长**按列处理**和**结构化文本**。以下是工程中最实用的技巧。

## 介绍

以列单位进行字符处理，默认情况下按照 TAB 或空格对文件进行拆分。

- 格式
  - `awk 参数 '脚本语句' 带操作文件`。例子：`ps aux | awk '{print $3}'` 打印进程信息中的第三列
  - `awk 参数 -f '脚本文件' 带操作文件` 
- 变量
  - `print`: 打印输出变量，默认打印数据后自动会换行。
  - `printf`: 类似于 C 语言中的 printf 函数用法。  
  - 两个特殊的条件：BEGIN、END。例子：统计一个文件中的所有空格数 `awk '/^ *$/ {count=count+1} END {print count}'
    test.txt`
  - 常用内建变量
    - FILENAME：当前输入文件的文件名
    - NR：当前行的行号，R(record)
    - NF：当前行所拥有的的列数，F(field)
    - OFS：输出字段的列分隔符，默认是空格
    - FS：输入文件的列分隔符，默认是空格和 TAB
    - ORS：输出字段的行分隔符，默认是换行符
    - RS：输入文件的行分隔符，默认是换行符
- 例子： 使用 awk 批量杀进程的命令：
    ```sh
    ps -ef | grep firefox | grep -v grep | awk '{print "kill -9 "$2}'|sh
    ```

    说明
    ```sh
    #列出了当前主机中运行的进程中包含firefox关键字的进程
    ps -ef | grep firefox | grep -v grep

    #列出了要kill掉这些进程的命令，并将之打印在了屏幕上 
    ps -ef | grep firefox | grep -v grep | awk '{print "kill -9 "$2}'

    #后面加上|sh后，则执行这些命令，进而杀掉了这些进程
    ps -ef | grep firefox | grep -v grep | awk '{print "kill -9 "$2}' | sh
    ```

## 一、核心概念：记录与字段

```bash
# 默认分隔符：空格或制表符
# $0 = 整行
# $1 = 第1列  $2 = 第2列  ...  $NF = 最后一列

echo "apple banana cherry" | awk '{print $2}'     # banana
echo "a b c" | awk '{print $NF}'                  # c (最后一列)
echo "a b c" | awk '{print $(NF-1)}'              # b (倒数第二列)
```

## 二、80% 工程场景用这些模式

### 场景 1：提取特定列（最常用）
```bash
# 从日志提取第3列（如IP地址、时间戳）
awk '{print $3}' access.log

# 提取多列，加分隔符
awk '{print $1, $3, $5}' data.txt
awk '{print $1":"$3":"$5}' data.txt

# 只显示包含特定内容的行
awk '/ERROR/ {print $2, $4}' app.log
```

### 场景 2：条件过滤（替代 grep 的 AND/OR）
```bash
# 第3列 > 100 且第5列包含 "FAIL"
awk '$3 > 100 && $5 ~ /FAIL/ {print $0}' data.txt

# 第2列等于 "ERROR" 或第4列等于 "WARN"
awk '$2 == "ERROR" || $4 == "WARN" {print}'

# 第1列不等于 "localhost"
awk '$1 != "localhost" {print}'
```

### 场景 3：统计与聚合（比 Excel 快）
```bash
# 统计文件行数
awk 'END {print NR}' file.txt

# 求第3列的总和
awk '{sum += $3} END {print "Total:", sum}'

# 求平均值
awk '{sum += $3; count++} END {print "Avg:", sum/count}'

# 分组统计（类似 SQL 的 GROUP BY）
awk '{count[$1]++} END {for(k in count) print k, count[k]}' access.log
```

### 场景 4：处理 CSV/TSV
```bash
# CSV 文件（逗号分隔）
awk -F',' '{print $2, $4}' data.csv

# TSV 文件（制表符分隔）
awk -F'\t' '{print $1, $3}' data.tsv

# 处理复杂 CSV（带引号的字段）
awk -F',' '{gsub(/"/, "", $0); print $1}'
```

### 场景 5：格式化输出（对齐表格）
```bash
# 左对齐，宽度20
awk '{printf "%-20s %-10s %5d\n", $1, $2, $3}'

# 右对齐
awk '{printf "%10s %20s\n", $1, $2}'

# 实际案例：ps 输出格式化
ps aux | awk '{printf "%-8s %6s %s\n", $1, $3, $11}'
```

## 三、工程实战技巧

### 技巧 1：多个模式匹配（AND/OR）
```bash
# 同时包含 shadow 和 desired
awk '/shadow/ && /desired/ {print NR, $0}' log.txt

# 包含 shadow 或 desired
awk '/shadow/ || /desired/ {print}'

# 第3列匹配正则
awk '$3 ~ /^error/i {print}'  # 忽略大小写
```

### 技巧 2：处理时间范围
```bash
# 过滤 10:00:00 到 11:00:00 的日志
awk '$2 >= "10:00:00" && $2 <= "11:00:00"' app.log

# 计算时间差（假设第2列是时间戳）
awk '{time[NR]=$2} NR>1 {print $2 - time[NR-1]}' log.txt
```

### 技巧 3：去重（类似 sort -u）
```bash
# 保留第一次出现的顺序
awk '!seen[$1]++ {print}' file.txt

# 统计重复次数
awk '{count[$0]++} END {for(ln in count) print count[ln], ln}' file.txt
```

### 技巧 4：内置变量速查
```bash
NR  # 当前行号（总行数）
NF  # 当前行的字段数
$NF # 最后一列的值
FS  # 输入分隔符（默认空格）
OFS # 输出分隔符（默认空格）
RS  # 输入记录分隔符（默认换行）
ORS # 输出记录分隔符（默认换行）
```

### 技巧 5：BEGIN/END 块
```bash
# BEGIN：处理第一行前执行（设置分隔符、打印表头）
# END：处理最后一行后执行（打印统计）

awk 'BEGIN {FS=","; print "=== Report ==="} 
     {total += $3} 
     END {print "Total:", total; print "Lines:", NR}' data.csv
```

## 四、真实工程案例

### 案例 1：分析 Nginx 日志
```bash
# 统计每个 IP 的访问次数
awk '{print $1}' access.log | sort | uniq -c | sort -rn | head -10

# 更优雅的 awk 版本（无需 sort/uniq）
awk '{ip[$1]++} END {for(k in ip) print ip[k], k}' access.log | sort -rn | head -10

# 统计状态码分布
awk '{code[$9]++} END {for(c in code) print c, code[c]}' access.log
```

### 案例 2：监控系统性能
```bash
# CPU 使用率（从 top 输出）
top -bn1 | awk '/%Cpu/ {print "CPU User:", $2, "% System:", $4}'

# 内存使用（从 /proc/meminfo）
awk '/MemTotal/ {total=$2} /MemAvailable/ {avail=$2} 
     END {printf "Used: %.1f%%\n", (total-avail)/total*100}' /proc/meminfo
```

### 案例 3：处理多行记录
```bash
# 处理以 "---" 分隔的多行记录
awk 'BEGIN {RS="---\n"} /ERROR/ {print "Found in record", NR, $0}' log.txt

# 合并被截断的行
awk '/^[[:space:]]/ {printf "%s", $0} !/^[[:space:]]/ {print ""; printf "%s", $0}' file.txt
```

### 案例 4：实时监控日志
```bash
# 实时 tail 日志并提取关键信息
tail -f app.log | awk '/ERROR/ {print strftime("%H:%M:%S"), $0}'

# 监控特定列的异常值
tail -f metrics.log | awk '$3 > 90 {print "ALERT: High value", $3, "at line", NR}'
```

## 五、常见陷阱与解决

### 陷阱 1：变量引用
```bash
# ❌ 错误：Shell 变量在单引号内无效
name="error"
awk '/$name/ {print}' log.txt  # 不会工作

# ✅ 正确：双引号或 -v 传参
awk "/$name/ {print}" log.txt
awk -v pat="$name" '$0 ~ pat {print}' log.txt
```

### 陷阱 2：比较数字 vs 字符串
```bash
# 数字比较
awk '$3 > 10'          # 正确
awk '$3 > "10"'        # 字符串比较，可能出错

# 字符串匹配
awk '$2 == "OK"'       # 正确
```

### 陷阱 3：字段分隔符
```bash
# 多个连续空格会被当做一个
echo "a  b   c" | awk '{print $2}'  # b

# 指定多个分隔符
awk -F'[ ,:]+' '{print $2}'  # 空格、逗号、冒号都当分隔符
```

## 六、学习路线建议

1. **先掌握**：`{print $n}`、`/pattern/`、条件判断、NR/NF
2. **再掌握**：BEGIN/END、数组、printf 格式化
3. **进阶**：自定义函数、多文件处理、getline

## 快速参考卡片

```bash
# 常用组合
awk '{print $1}'                     # 取第一列
awk 'NR>1 && NR<10'                  # 2-9行
awk 'length($0)>80'                  # 长行
awk '!a[$0]++'                       # 去重
awk '{sum+=$1} END{print sum/NR}'    # 平均值
awk 'NF>0'                           # 过滤空行
awk 'END{print NR}'                  # 计数
```

**记住**：awk 最大的价值在于**不需要写循环**就能处理列数据，30 秒能解决的问题没必要写 Python 脚本。

