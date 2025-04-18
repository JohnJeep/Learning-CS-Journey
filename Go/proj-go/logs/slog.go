/*
 * @Author: JohnJeep
 * @Date: 2025-03-18 16:25:50
 * @LastEditors: JohnJeep
 * @LastEditTime: 2025-04-18 17:02:33
 * @Description: Go 1.21 版本标准库中推出了新的日志库：log/slog
 * Copyright (c) 2025 by John Jeep, All Rights Reserved.
 */

package main

import (
	"fmt"
	"log/slog"
	"os"
	"path/filepath"
)

func defaultSlog() {
	logger := slog.Default()
	logger.Info("Good", "user", os.Getenv("USER"))
	slog.Info("Hello word!")
}

func JsonSlog() {
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	logger.Info("exectue json slog")
}

func custom_logger_attr() {
	level := slog.LevelDebug
	handler := slog.NewTextHandler(os.Stderr, &slog.HandlerOptions{
		Level:     level,
		AddSource: true,
		ReplaceAttr: func(_ []string, attr slog.Attr) slog.Attr {
			if attr.Key == slog.SourceKey {
				source := attr.Value.Any().(*slog.Source)
				source.File = filepath.Base(source.File)
			}
			return attr
		},
	})

	logger := slog.New(handler)
	logger.Info("Hello world", "user", os.Getenv("USER"))
	logger.Debug("This is a debug message", "key1", "value1")
	logger.Info("This is an info message", "key2", "value2")
	logger.Warn("This is a warning message", "key3", "value3")
	logger.Error("This is an error message", "key4", "value4", "error", fmt.Errorf("some error occurred"))
}

func main() {
	// defaultSlog()
	// JsonSlog()
	custom_logger_attr()
}
