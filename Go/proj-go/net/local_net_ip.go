package main

import (
	"fmt"
	"net"
	"strings"
)

func main() {
	ips, err := GetLocalNet()
	if err != nil {
		fmt.Println("get local ip error:", err)
	}

	for _, ip := range ips {
		fmt.Println("machine address:", ip)
	}
}

func GetLocalNet() ([]string, error) {
	var ips []string

	interfaces, err := net.Interfaces()
	if err != nil {
		return ips, fmt.Errorf("获取网络接口信息失败: %v", err)
	}

	for _, iface := range interfaces {
		if iface.Flags&net.FlagUp == 0 || iface.Flags&net.FlagLoopback != 0 {
			continue // 跳过未启用或是回环网卡
		}

		addrs, err := iface.Addrs()
		if err != nil {
			continue
		}

		for _, addr := range addrs {
			ipNet, ok := addr.(*net.IPNet)
			if !ok || ipNet.IP.To4() == nil {
				continue // 跳过非 IPv4 地址和无效地址
			}

			// 过滤掉虚拟网卡的地址
			if !isVirtualNetCard(iface.Name) {
				fmt.Printf("Interface: %s, IP Address: %s\n", iface.Name, ipNet.IP.String())
				ips = append(ips, ipNet.IP.String())
			}
		}
	}

	return ips, nil
}

// 判断是否是虚拟网卡
func isVirtualNetCard(ifaceName string) bool {
	virtualKeywords := []string{"virtual", "vmware", "vbox", "docker", "veth", "tun", "tap"}
	nameLower := strings.ToLower(ifaceName)

	for _, keyword := range virtualKeywords {
		if strings.Contains(nameLower, keyword) {
			return true
		}
	}

	return false
}
