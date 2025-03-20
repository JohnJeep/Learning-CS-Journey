// 简单的 http server
// 通过 http.HandleFunc 注册一个处理函数，然后通过 http.ListenAndServe 启动一个 http server

package main

import (
	"fmt"
	"net/http"
)

func handler(w http.ResponseWriter, r *http.Request) {
	fmt.Fprintf(w, "I'm http server")
}

func main() {
	http.HandleFunc("/", handler)
	http.ListenAndServe(":8081", nil)
}
