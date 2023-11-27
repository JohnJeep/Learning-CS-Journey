package main

import (
	"encoding/json"
	"fmt"
)

// Subscriber 结构体用于解析JSON中的订阅者信息
type Subscriber struct {
	Identity struct {
		Pid struct {
			Address string `json:"address"`
			ID      string `json:"id"`
		} `json:"pid"`
	} `json:"identity"`
}

// Subscribers 结构体用于解析包含订阅者信息的JSON
type Subscribers struct {
	Subscribers []Subscriber `json:"subscribers"`
}

func main() {
	subscriber := Subscriber{
		Identity: struct {
			Pid struct {
				Address string `json:"address"`
				ID      string `json:"id"`
			} `json:"pid"`
		}{
			Pid: struct {
				Address string `json:"address"`
				ID      string `json:"id"`
			}{
				Address: "127.0.0.1:10086",
				ID:      "partition-activator/Test$d",
			},
		},
	}
	data := Subscribers{
		Subscribers: []Subscriber{subscriber},
	}

	fmt.Printf("转化为结构体后的数据: %+v\n", data)
	encodedData, err := json.Marshal(data)
	if err != nil {
		fmt.Println("编码 JSON 失败:", err)
		return
	}
	fmt.Println("[]byte:", encodedData)
	fmt.Printf("转化为JSON类型数据:%s\n", string(encodedData))

	var subscribersData Subscribers
	err = json.Unmarshal([]byte(encodedData), &subscribersData)
	if err != nil {
		fmt.Println("解码 JSON 失败:", err)
		return
	}
	fmt.Println("sub:", subscribersData)
}
