// 整型转化为字符串，比使用 fmt.Sprintf() 效率高，与  strconv.Itoa() 效率差不多
package main

import (
	"fmt"
	"strconv"
)

func main() {
	// v := int64(100)
	var v int64 = 200

	s10 := strconv.FormatInt(v, 10)
	fmt.Printf("type: %T, value: %v\n", s10, s10)

	// 16进制数转 string
	s16 := strconv.FormatInt(v, 16)
	fmt.Printf("type: %T, valeu: %v\n", s16, s16)
}
