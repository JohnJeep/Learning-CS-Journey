// struct 类型数据序列化 json 字符串
// 场景：客户端请求Go服务端，服务将数据类型为 struct 的结构体返回客户端
//       服务端的struct中数据的属性首字母是大写的，客户端处理不方便，
//       客户端只接受首字母为小写格式的 json 字符串

package main

import (
	"encoding/json"
	"fmt"
)

type Monster struct {
	// 字段未 tag
	// Name  string
	// Blood int
	// Skill string

	// 字段使用 tag 后
	Name  string `json:"name"`
	Blood int    `json:"blood"`
	Skill string `josn:"skill"`
}

func main() {
	monster := Monster{"剑魔", 3867, "赐死剑气之大灭"}
	fmt.Println("monster:", monster)
	jsonByte, err := json.Marshal(monster)
	if err != nil {
		fmt.Println("Struct 序列化为 json失败")
	} else {
		jsonStr := string(jsonByte)
		fmt.Println("json str:", jsonStr)
	}
}
