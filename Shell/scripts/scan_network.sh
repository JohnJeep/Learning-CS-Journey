#!/bin/bash
#
# 查找指定网段活跃的IP
#

# 指定网段，例如 172.26.136
network="172.26.136"

# 输出文件
output_file="active_ips.txt"

# 先清空文件内容，防止之前的结果残留
> "$output_file"

# 遍历 1-254 IP 范围
for i in {1..254}; do
  ip="$network.$i"

  # 并行发送 1 个 ICMP 请求，超时 0.2 秒
  (
    if ping -c 1 -W 0.2 "$ip" &> /dev/null; then
      # 如果 ping 成功，直接写入 IP
      echo "$ip is active" >> "$output_file"
    fi
  ) &
done

# 等待所有后台进程完成
wait

echo "Active IPs have been written to $output_file"
