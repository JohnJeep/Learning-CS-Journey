package main

import (
	"log"
	"tcp-demo/client/client"
)

func main() {
	c := client.NewClient("CLIENT_001")
	if err := c.Connect("localhost:8080"); err != nil {
		log.Fatal(err)
	}
	defer c.Close()

	c.CreateDevice()
	c.UpdateDevice()
	c.StartHeartbeat()

	select {}
}
