/*
 * @Author: JohnJeep
 * @Date: 2025-04-24 09:31:49
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-24 09:41:19
 * @Description: storage interface
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */
package main

import "fmt"

type Storage interface {
	Save(data string) error
	Load() (string, error)
}

type FileStorage struct{}

func (fs *FileStorage) Save(data string) error {
	// Implement file saving logic here
	fmt.Println("Saving data to file...")
	return nil
}

func (fs *FileStorage) Load() (string, error) {
	// Implement file loading logic here
	fmt.Println("Loading data from file...")
	return "fs", nil
}

type DatabaseStorage struct{}

func (ds *DatabaseStorage) Save(data string) error {
	// Implement database saving logic here
	fmt.Println("Saving data to database...")
	return nil
}

func (ds *DatabaseStorage) Load() (string, error) {
	// Implement database loading logic here
	fmt.Println("Loading data from database...")
	return "database", nil
}

// business logic depends on the interface
type App struct {
	storage Storage
}

// interface as a parameter, allowing for different implementations
// returning a pointer to the App struct
// this is a constructor function
func NewApp(storage Storage) *App {
	return &App{storage: storage}
}

// Run method to demonstrate the use of the storage interface
func (a *App) Run() {
	data, _ := a.storage.Load()
	fmt.Println("Loaded data:", data)
	// Do something with the loaded data

	a.storage.Save("data")
}

func main() {
	fileStorage := &FileStorage{}
	app1 := NewApp(fileStorage)
	app1.Run()

	databaseStorage := &DatabaseStorage{}
	app2 := NewApp(databaseStorage)
	app2.Run()
}
