<!--
 * @Author: JohnJeep
 * @Date: 2021-04-26 21:05:36
 * @LastEditTime: 2021-04-26 22:16:24
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
# 15-三数之和

# 题目描述
给你一个包含 n 个整数的数组 nums，判断 nums 中是否存在三个元素 a，b，c ，使得 a + b + c = 0 ？请你找出所有和为 0 且不重复的三元组。

注意：答案中不可以包含重复的三元组。

 
示例 1：
```
输入：nums = [-1,0,1,2,-1,-4]
输出：[[-1,-1,2],[-1,0,1]]
```

示例 2：
```
输入：nums = []
输出：[]
```

示例 3：
```
输入：nums = [0]
输出：[]
```
# 题解
排序 + 双指针

先对原数组进行排序，在排序好序的数组中先固定一个值，然后将该题转化为求两数之和为一个固定值。

# 代码
```cpp
class Solution {
public:
    vector<vector<int>> threeSum(vector<int>& nums) {
        std::sort(nums.begin(), nums.end());     // 递增排序  

        int len = nums.size();
        vector<vector<int>> arr;                // 保存所有不重复的三元组的结果

        if (len < 3) {  // 特判
            return {};
        }
        for (int i = 0; i < len; i++) {         // 固定第一个数，转化为求两数之和
            if (nums[i] > 0) {                  // 第一个数大于 0，后面都是递增正数，不可能相加为零了
                return arr;
            }
            if (i > 0 && (nums[i] == nums[i - 1])) {  
                continue;                       // 去重：如果此数已经选取过，跳过
            }

            // 双指针在nums[i]后面的区间中寻找和为 0-nums[i] 的另外两个数
            int left = i + 1;
            int right = len - 1;

            while (left < right) {
                if (nums[left] + nums[right] > -nums[i]) {
                    right--;                     // 两数之和太大，右指针左移
                }
                else if (nums[left] + nums[right] < -nums[i]) {
                    left++;                      // 两数之和太小，左指针右移
                } 
                else {
                    // 找到一个和为零的三元组，添加到结果中，左右指针内缩，继续寻找
                    arr.push_back(vector<int>{nums[i], nums[left], nums[right]});
                    left++;
                    right--;

                    // 去重：第二个数和第三个数也不重复选取
                    // 例如：[-4,1,1,1,2,3,3,3], i=0, left=1, right=5
                    while (left < right && nums[left] == nums[left - 1]) {
                        left++;
                    }
                    while (left <right && nums[right] == nums[right + 1]) {
                        right--;
                    }
                }
            }
        }
        return arr;
    }
};
```

# 参考
- [Terry 解三数之和的思路](https://leetcode-cn.com/problems/3sum/solution/jian-ji-ming-liao-qing-xi-yi-dong-zhu-ji-c8jb/)