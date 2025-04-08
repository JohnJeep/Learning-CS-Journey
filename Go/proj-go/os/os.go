package main

import (
	"fmt"
	"os"
)

func main() {
	currDir, err := os.Getwd()
	if err != nil {
		fmt.Println("get current dir failed:", err)
	}
	fmt.Println("path:", currDir)
	entries, err := os.ReadDir(currDir)

	for _, entry := range entries {
		info, err := entry.Info()
		if err != nil {
			fmt.Println("get entry info failed:", err)
			continue
		}
		k := info.Name()
		v := info.IsDir()
		if err != nil {
			fmt.Println("get entry info failed:", err)
			continue
		}
		fmt.Println("key:", k, "val:", v, "mode:", info.Mode(), "size:", info.Size())

		fileInfo, _ := os.Stat(k)
		fmt.Println("++++", fileInfo.Name(), fileInfo.Sys())
	}
	fmt.Println("dir", entries)

}
