package main

import (
	"fmt"
	"os"
)

func main() {
	filePath := "os.go"
	fileInfo, err := os.Stat(filePath)
	if err != nil {
		if os.IsNotExist(err) {
			fmt.Printf("文件 %s 不存在\n", filePath)
		} else {
			fmt.Printf("获取文件信息时出错: %v\n", err)
		}
		return
	}

	// 输出文件信息
	fmt.Printf("文件名称: %s\n", fileInfo.Name())
	fmt.Printf("文件大小: %d 字节\n", fileInfo.Size())
	fmt.Printf("文件权限: %s\n", fileInfo.Mode())
	fmt.Printf("最后修改时间: %s\n", fileInfo.ModTime())
	fmt.Printf("是否为目录: %v\n", fileInfo.IsDir())
}
