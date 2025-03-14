package main

import (
	"database/sql"
	"fmt"
	"strconv"

	_ "github.com/go-sql-driver/mysql"
)

func main() {
	dsn := "mysqladmin:Mysql2020Admin@tcp(loalhost:3306)/iotplat"

	db, err := sql.Open("mysql", dsn)
	if err != nil {
		fmt.Println("Failed to open database connection:", err)
		return
	}
	defer db.Close()

	err = db.Ping()
	if err != nil {
		fmt.Println("Failed to ping database:", err)
		return
	}

	// 查询当前会话的 sort_buffer_size
	var name string
	var valueStr string
	err = db.QueryRow("SHOW VARIABLES LIKE 'sort_buffer_size'").Scan(&name, &valueStr)
	if err != nil {
		fmt.Println("Failed to query session sort_buffer_size:", err)
		return
	}

	sessionSortBufferSize, err := strconv.Atoi(valueStr)
	if err != nil {
		fmt.Println("Failed to convert sort_buffer_size to int:", err)
		return
	}
	fmt.Printf("Session sort_buffer_size: %d bytes\n", sessionSortBufferSize)

	// 查询全局的 sort_buffer_size
	var globalValueStr string
	err = db.QueryRow("SHOW GLOBAL VARIABLES LIKE 'sort_buffer_size'").Scan(&name, &globalValueStr)
	if err != nil {
		fmt.Println("Failed to query global sort_buffer_size:", err)
		return
	}

	globalSortBufferSize, err := strconv.Atoi(globalValueStr)
	if err != nil {
		fmt.Println("Failed to convert global sort_buffer_size to int:", err)
		return
	}
	fmt.Printf("Global sort_buffer_size: %d bytes\n", globalSortBufferSize)
}
