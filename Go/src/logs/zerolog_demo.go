// Reference: 第三方库 zerolog 用法 https://www.cnblogs.com/dribs/p/17196295.html

package main

import (
	"fmt"
	"os"
	"strings"
	"time"

	"github.com/rs/zerolog"
	"github.com/rs/zerolog/log"
)

func test01() {
	//仅仅打印输出字符串
	log.Print("Hello word!")

	// 时间输出为 unix 格式
	// output: {"level":"debug","time":1687915081,"message":"Good!!!"}
	zerolog.TimeFieldFormat = zerolog.TimeFormatUnix
	log.Print("Good!!!")
}

// 日志输出带有不同的颜色，方便阅读
// Notice: 用 ConsoleWriter 性能不够理想，建议只在开发环境下使用
func test02() {
	output := zerolog.ConsoleWriter{Out: os.Stderr, TimeFormat: time.RFC3339}

	// 制输出的级别、信息、字段名、字段值的格式
	output.FormatLevel = func(i interface{}) string {
		return strings.ToUpper(fmt.Sprintf("| %-6s|", i))
	}
	output.FormatMessage = func(i interface{}) string {
		return fmt.Sprintf("***%s****", i)
	}
	output.FormatFieldName = func(i interface{}) string {
		return fmt.Sprintf("%s:", i)
	}
	output.FormatFieldValue = func(i interface{}) string {
		return strings.ToUpper(fmt.Sprintf("%s", i))
	}

	logger := log.Output(output).With().Timestamp().Logger()
	logger.Info().Str("foo", "bar").Msg("hello world")
}

// 自定义 logger
// 创建子Logger时带入Caller()选项
// 日志输出中带有文件名和行号
func test03() {
	logger := zerolog.New(os.Stderr).With().Caller().Logger()
	logger.Info().Msg("Google nice!!!")
}

// 日志采样
func test04() {
	sampled := log.Sample(&zerolog.BasicSampler{N: 10})

	for i := 0; i < 20; i++ {
		sampled.Info().Msg("will be logged every 10 message")
	}
}

func test05() {
	color := map[string]string{"Red": "#FFFFF"}
	if val, exist := color["Red"]; exist {
		fmt.Println(val)
	}

}

func main() {
	test01()
	test02()
	test03()
	test04()
	test05()

}
