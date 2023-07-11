#!/bin/bash
# 指定时间段内使用 ping 命令 ping 指定的某台主机

# 设置目标主机的 IP 地址或主机名
target_host="172.26.136.48"

# 获取当前时间（包括年、月、日、小时、分钟、秒）
current_time=$(date +'%Y-%m-%d %H:%M:%S')
echo $current_time

# 定义要执行的命令
# ping_command="ping -c 1 $target_host | grep 'time=' | cut -d '=' -f 4"
# ping_command="ping -c 1 $target_host | grep 'time=' | sed \"s/^/$(date +'%H:%M') /\""
ping_command="ping -c 1 $target_host | grep 'time=' | sed \"s/^/$current_time /\""


# 检查当前时间是否在指定时间范围内
if [[ "$current_time" > "2023-07-10 07:37" && "$current_time" < "2023-07-30 19:38" ]]; then
    echo "当前时间：$current_time"
    echo "正在 ping $target_host ..."

    # 执行 ping 命令并将结果写入文件
    eval $ping_command > ping_result.txt
    echo "已将结果写入 ping_result.txt 文件。"
else
    echo "当前时间不在指定时间范围内。"
fi
