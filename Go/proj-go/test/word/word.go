package word

import (
	"unicode"
)

// 判断输入的但是是不是回文
func IsPalindrome(s string) bool {
	for i := range s {
		if s[i] != s[len(s)-1-i] {
			return false
		}
	}
	return true
}

// 改进版本
func IsPalindrome02(s string) bool {
	var letters []rune
	for _, v := range s {
		if unicode.IsLetter(v) {
			letters = append(letters, unicode.ToLower(v))
		}
	}

	for i := range letters {
		if letters[i] != letters[len(letters)-1-i] {
			return false
		}
	}
	return true
}

func IsPalindrome03(s string) bool {
	var letters []rune
	for _, v := range s {
		if unicode.IsLetter(v) {
			letters = append(letters, unicode.ToLower(v))
		}
	}

	count := len(letters) / 2
	for i := 0; i < count; i++ {
		if letters[i] != letters[len(letters)-1-i] {
			return false
		}
	}
	return true
}

// 在开始为每个字符预先分配一个足够大的数组， 这样就可以避免在append
// 调用时可能会导致内存的多次重新分配。 声明一个letters数组变量， 并指定合适的大小
func IsPalindrome04(s string) bool {
	letters := make([]rune, 0, len(s))
	for _, v := range s {
		if unicode.IsLetter(v) {
			letters = append(letters, unicode.ToLower(v))
		}
	}

	count := len(letters) / 2
	for i := 0; i < count; i++ {
		if letters[i] != letters[len(letters)-1-i] {
			return false
		}
	}
	return true
}
