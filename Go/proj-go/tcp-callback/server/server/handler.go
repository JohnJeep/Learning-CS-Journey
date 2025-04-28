package server

import (
	"bufio"
	"encoding/json"
	"log"
	"net"
	"tcp-demo/common"
)

type Callback func(*common.Message, net.Conn)

var callbacks = make(map[string]Callback)

func HandleClient(conn net.Conn) {
	defer conn.Close()

	scanner := bufio.NewScanner(conn)
	for scanner.Scan() {
		var msg common.Message
		if err := json.Unmarshal(scanner.Bytes(), &msg); err != nil {
			log.Println("Decode error:", err)
			return
		}

		if handler, exists := callbacks[msg.Action]; exists {
			handler(&msg, conn)
		} else {
			log.Println("Unknown action:", msg.Action)
		}
	}

	if err := scanner.Err(); err != nil {
		log.Println("Scanner error:", err)
	}
}

func init() {
	callbacks[common.RegisterAction] = handleRegister
	callbacks[common.CreateDeviceAction] = handleCreateDevice
	callbacks[common.UpdateDeviceAction] = handleUpdateDevice
	callbacks[common.HeartbeatAction] = handleHeartbeat
}

func handleRegister(msg *common.Message, conn net.Conn) {
	log.Printf("Client registered with code: %s\n", msg.Code)
	sendResponse(conn, common.Response{Status: 200, Message: "Registration successful"})
}

func handleCreateDevice(msg *common.Message, conn net.Conn) {
	log.Println("Creating device with payload:", msg.Payload)
	sendResponse(conn, common.Response{Status: 200, Message: "Device created", Data: map[string]string{"id": "123"}})
}

func handleUpdateDevice(msg *common.Message, conn net.Conn) {
	log.Println("Updating device with payload:", msg.Payload)
	sendResponse(conn, common.Response{Status: 200, Message: "Device updated"})
}

func handleHeartbeat(msg *common.Message, conn net.Conn) {
	log.Println("Received heartbeat")
}

func sendResponse(conn net.Conn, resp common.Response) {
	data, _ := json.Marshal(resp)
	conn.Write(append(data, '\n'))
}
