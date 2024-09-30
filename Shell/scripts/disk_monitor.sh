#!/bin/bash

# 监控磁盘空间的使用率。当每个磁盘分区使用空间超过90%时，通过sendEmail来发送告警邮件

# 设置邮件相关信息
recipient="your_email@example.com"  # 收件人邮箱
subject="Disk Space Alert"            # 邮件主题
body="The following partitions are using more than 90% of disk space:\n\n"  # 邮件内容

# 获取所有磁盘分区的使用情况
# 使用 df 命令，获取文件系统、使用率和挂载点
df -h --output=source,pcent,target | tail -n +2 | while read -r line; do
  # 解析每一行的输出
  partition=$(echo "$line" | awk '{print $1}')   # 分区名
  usage=$(echo "$line" | awk '{print $2}' | tr -d '%')  # 使用率，去掉 %
  mount_point=$(echo "$line" | awk '{print $3}')  # 挂载点
  
  # 检查使用率是否超过 90%
  if [ "$usage" -gt 90 ]; then
    # 如果超过 90%，将该信息添加到邮件内容
    body+="$partition ($mount_point): $usage%\n"
  fi
done

# 如果有任何分区使用率超过 90%，发送告警邮件
if [[ "$body" != "The following partitions are using more than 90% of disk space:\n\n" ]]; then
  # 发送邮件
  echo -e "$body" | sendEmail -f your_email@example.com -t "$recipient" -u "$subject" -s smtp.example.com:587 -xu your_username -xp your_password
fi
