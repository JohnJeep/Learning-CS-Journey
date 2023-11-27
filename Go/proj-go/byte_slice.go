package main

import (
	"encoding/base64"
	"fmt"
)

func main() {
	// 要编码的原始数据
	originalData := []byte("Hello, World!")

	// 进行Base64编码
	encodedData := base64.StdEncoding.EncodeToString(originalData)
	fmt.Println("Base64 编码结果:", encodedData)

	// 进行Base64解码
	decodedData, err := base64.StdEncoding.DecodeString(encodedData)
	if err != nil {
		fmt.Println("解码时发生错误:", err)
		return
	}
	fmt.Println("Base64 解码结果:", string(decodedData))
}
