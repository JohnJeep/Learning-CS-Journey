package client

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"tcp-demo/common"
	"time"
)

type Client struct {
	ID        string
	conn      net.Conn
	callbacks map[string]func(common.Response)
}

func NewClient(id string) *Client {
	return &Client{
		ID:        id,
		callbacks: make(map[string]func(common.Response)),
	}
}

func (c *Client) Connect(addr string) error {
	conn, err := net.Dial("tcp", addr)
	if err != nil {
		return err
	}
	c.conn = conn

	// 发送注册消息
	msg := common.Message{
		Action: common.RegisterAction,
		Code:   c.ID,
	}
	c.sendMessage(msg)

	// 启动响应处理
	go c.handleResponses()

	return nil
}

func (c *Client) sendMessage(msg common.Message) {
	data, _ := json.Marshal(msg)
	c.conn.Write(append(data, '\n'))
}

func (c *Client) handleResponses() {
	scanner := bufio.NewScanner(c.conn)
	for scanner.Scan() {
		var resp common.Response
		if err := json.Unmarshal(scanner.Bytes(), &resp); err != nil {
			log.Println("Decode error:", err)
			continue
		}
		log.Printf("Received response: %+v\n", resp)
	}
}

func (c *Client) CreateDevice() {
	msg := common.Message{
		Action:  common.CreateDeviceAction,
		Payload: map[string]string{"name": "Device1"},
	}
	c.sendMessage(msg)
}

func (c *Client) UpdateDevice() {
	msg := common.Message{
		Action:  common.UpdateDeviceAction,
		Payload: map[string]string{"id": "123", "name": "UpdatedDevice"},
	}
	c.sendMessage(msg)
}

func (c *Client) StartHeartbeat() {
	ticker := time.NewTicker(3 * time.Second)
	go func() {
		for range ticker.C {
			msg := common.Message{Action: common.HeartbeatAction}
			c.sendMessage(msg)
			log.Println("Sent heartbeat")
		}
	}()
}

func (c *Client) Close() {
	c.conn.Close()
}
