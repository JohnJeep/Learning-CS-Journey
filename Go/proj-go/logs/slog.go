// Go 1.21 版本标准库中推出了新的日志库：log/slog

package main

import (
	"log/slog"
	"os"
)

func defaultSlog() {
	// 显示得到logger
	logger := slog.Default()
	logger.Info("Good", "user", os.Getenv("USER"))

	slog.Info("Hello word!")
}

func JsonSlog() {
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	logger.Info("exectue json slog")
}

func main() {
	JsonSlog()
}
