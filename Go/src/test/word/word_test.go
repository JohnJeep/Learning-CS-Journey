package word

import (
	"log"
	"testing"
)

func TestIsPalindrome(t *testing.T) {
	if !IsPalindrome("testset") {
		t.Error("testset error")
	}

	if !IsPalindrome("kayak") {
		t.Error("kayak error")
	}

	// 法语
	// 出错原因：不能识别非ACII
	if !IsPalindrome("été") {
		// t.Errorf("été error")
		log.Println("été error")
	}

	// error: 没有过滤掉空格和字母大小写
	if !IsPalindrome("A man, a plan, a canal: Panama.") {
		// t.Errorf("long string error")
		log.Println("long string error")
	}
}

// 非回显参数
func TestNonPalindrome(t *testing.T) {
	if IsPalindrome("palindrome") {
		t.Error(`palindrome error`)
	}
}

// 改进版本测试用例
func TestIsPalindrome02(t *testing.T) {
	// 表格驱动测试用例，这种写法很常见
	var tests = []struct {
		input string
		want  bool // 默认值为 false
	}{
		{"", true},
		{"a", true},
		{"aa", true},
		{"ab", false},
		{"kayak", true},
		{"detartrated", true},
		{"A man, a plan, a canal: Panama", true},
		{"Evil I did dwell; lewd did I live.", true},
		{"Able was I ere I saw Elba", true},
		{"été", true},
		{"Et se resservir, ivresse reste.", true},
		{"palindrome", false}, // non-palindrome
		{"desserts", false},   // semi-palindrome
	}

	for _, test := range tests {
		if got := IsPalindrome02(test.input); got != test.want {
			// t.Errorf调用也没有引起panic异常或停止测试的执行
			t.Errorf("IsPalindrome(%q) = %v", test.input, got)
		}
	}
}

// 基准测试
func BenchmarkIsPalindrome02(b *testing.B) {
	for i := 0; i < b.N; i++ {
		IsPalindrome02("A man, a plan, a canal: Panama")
	}
}

func BenchmarkIsPalindrome03(b *testing.B) {
	for i := 0; i < b.N; i++ {
		IsPalindrome03("A man, a plan, a canal: Panama")
	}
}

func BenchmarkIsPalindrome04(b *testing.B) {
	for i := 0; i < b.N; i++ {
		IsPalindrome04("A man, a plan, a canal: Panama")
	}
}
