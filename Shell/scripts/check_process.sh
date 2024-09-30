#!/bin/bash
#
# 根据PID过滤进程所有信息，然后给出进程状态和占用资源信息
#

# 检查输入参数
if [ $# -ne 1 ]; then
  echo "Usage: $0 <PID>"
  exit 1
fi

PID=$1

# 获取进程信息
process_info=$(ps -p "$PID" -o pid,comm,state,%cpu,%mem,vsz,rss)

# 检查进程是否存在
if [ -z "$process_info" ]; then
  echo "No process found with PID $PID"
  exit 1
fi

# 输出标题
echo "Process information for PID $PID:"
echo "-------------------------------------"
echo "pid:    Process ID"
echo "comm:   Command name"
echo "state:  Process state"
echo "%cpu:   CPU usage percentage"
echo "%mem:   Memory usage percentage"
echo "vsz:    Virtual memory size (KB)"
echo "rss:    Resident memory size (KB)"
echo "-------------------------------------"

# 输出进程信息
echo "$process_info"
