<!--
 * @Author: JohnJeep
 * @Date: 2021-04-19 20:38:31
 * @LastEditTime: 2021-04-20 21:38:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
# 3-无重复字符串的最长子串

# 题目描述
**给定一个字符串，请你找出其中不含有重复字符的最长子串的长度。**

示例 1:
```
输入: s = "abcabcbb"
输出: 3 
解释: 因为无重复字符的最长子串是 "abc"，所以其长度为 3。
```

示例 2:
```
输入: s = "bbbbb"
输出: 1
解释: 因为无重复字符的最长子串是 "b"，所以其长度为 1。
```

示例 3:
```
输入: s = "pwwkew"
输出: 3
解释: 因为无重复字符的最长子串是 "wke"，所以其长度为 3。
     请注意，你的答案必须是 子串 的长度，"pwke" 是一个子序列，不是子串。
```

示例 4:
```
输入: s = ""
输出: 0
```


# 思路
采用滑动窗口的思想。



# 题解
- C++版本实现
```cpp
class Solution {
public:
    int lengthOfLongestSubstring(string s) {
        int res = 0; // 记录最长无重复子串的长度
        int left = -1; // 指向该无重复子串左边的起始位置的前一个
        unordered_map<int, int> m;
        for (int i = 0; i < s.size(); ++i) {            
            // 判断当前字符是否在HashMap中已存在
            // 若当前字符已在HashMap中，且映射值大于left的话，更新 left 为当前映射值
            if (m.count(s[i]) && m[s[i]] > left) {
                left = m[s[i]];  // 若存在，移除之前在窗口出现的字符
            }            
            m[s[i]] = i;  // 更新映射值为当前坐标i,这样保证了 left 始终为当前边界的前一个位置
            res = max(res, i - left);   // 计算窗口长度         
        }
        return res;
    }
};
```

# 复杂度
时间复杂度：O(N)

空间复杂度：O(N)


# 参考
- [无重复字符的最长子串 c++实现三种解法 多重循环，hashmap优化，桶优化](https://leetcode-cn.com/problems/longest-substring-without-repeating-characters/solution/wu-zhong-fu-zi-fu-de-zui-chang-zi-chuan-cshi-xian-/)
- [滑动窗口](https://leetcode-cn.com/problems/longest-substring-without-repeating-characters/solution/hua-dong-chuang-kou-by-powcai/)
- [[LeetCode] 3. Longest Substring Without Repeating Characters](https://github.com/grandyang/leetcode/issues/3)