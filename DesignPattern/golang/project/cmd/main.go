package main

import (
	"database/sql"
	"fmt"
	"project/internal/config"
	"project/internal/repository/mysql"
	"project/internal/service"

	_ "github.com/go-sql-driver/mysql"
)

func main() {
	cfg := config.Load()

	dsn := fmt.Sprintf("%s:%s@tcp(%s:%d)/%s", cfg.Username, cfg.Password, cfg.Host, cfg.Port, cfg.Database)
	db, err := sql.Open("mysql", dsn)
	if err != nil {
		panic(err)
	}
	defer db.Close()

	repo := mysql.New(db)
	service := service.New(repo)

	user, _ := service.GetUser(1)
	fmt.Printf("User: %v\n", user)
}
