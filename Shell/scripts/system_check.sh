
#!/bin/bash

# 设置颜色变量
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # 重置颜色

# 生成分隔线
separator() {
    echo -e "${BLUE}=======================================================${NC}"
}

# 系统基本信息检查
system_info() {
    echo -e "\n${GREEN}>>> 系统基本信息检查${NC}"
    separator
    echo "主机名       : $(hostname)"
    echo "操作系统版本 : $(cat /etc/redhat-release)"
    echo "内核版本     : $(uname -r)"
    echo "系统运行时间 : $(uptime | awk -F, '{print $1}')"
}

# CPU检查
cpu_check() {
    echo -e "\n${GREEN}>>> CPU检查${NC}"
    separator
    echo "CPU型号      : $(lscpu | grep 'Model name' | cut -d':' -f2 | xargs)"
    echo "物理核心数  : $(lscpu | grep 'Core(s)' | head -1 | awk '{print $4}')"
    echo "逻辑核心数  : $(nproc)"
    echo "当前负载    : $(uptime | awk -F 'average:' '{print $2}')"
}

# 内存检查
memory_check() {
    echo -e "\n${GREEN}>>> 内存检查${NC}"
    separator
    free -h | awk '
        /Mem/{
            print "总内存     : " $2
            print "已用内存   : " $3
            print "可用内存   : " $7
        }
        /Swap/{
            print "交换分区   : " $2
            print "已用交换   : " $3
        }'
}

# 硬盘检查
disk_check() {
    echo -e "\n${GREEN}>>> 硬盘检查${NC}"
    separator
    df -h | awk '
        BEGIN {
            print "挂载点\t\t总大小\t已用\t可用\t使用率"
        }
        /^\/dev/ {
            printf "%-15s %-6s %-6s %-6s %-4s\n", $6, $2, $3, $4, $5
        }' | column -t
}

# 网卡检查
network_check() {
    echo -e "\n${GREEN}>>> 网络接口检查${NC}"
    separator
    for interface in $(ip -o link show | awk -F': ' '{print $2}' | grep -v lo); do
        echo "接口名称  : $interface"
        echo "IP地址    : $(ip -o -4 addr show $interface | awk '{print $4}')"
        echo "MAC地址   : $(ip link show $interface | awk '/link\/ether/ {print $2}')"
        echo "连接速度  : $(ethtool $interface 2>/dev/null | grep Speed | awk '{print $2}')"
        echo "连接状态  : $(ip link show $interface | grep -o 'state [A-Z]*' | awk '{print $2}')"
        separator
    done
}

main() {
    clear
    echo -e "\n${YELLOW}====== 开始系统检查 ======${NC}"

    system_info
    cpu_check
    memory_check
    disk_check
    network_check

    echo -e "\n${YELLOW}====== 检查完成 ======${NC}"
}

main
