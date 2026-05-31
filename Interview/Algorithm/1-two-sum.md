<!--
 * @Author: JohnJeep
 * @Date: 2021-04-23 23:07:47
 * @LastEditTime: 2021-04-23 23:32:16
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
# 1-两数之和

# 题目描述
给定一个整数数组 nums 和一个整数目标值 target，请你在该数组中找出 和为目标值 的那 两个 整数，并返回它们的数组下标。

你可以假设每种输入只会对应一个答案。但是，数组中同一个元素在答案里不能重复出现。

你可以按任意顺序返回答案。

 

示例 1：
```
输入：nums = [2,7,11,15], target = 9
输出：[0,1]
解释：因为 nums[0] + nums[1] == 9 ，返回 [0, 1] 。
```

示例 2：
```
输入：nums = [3,2,4], target = 6
输出：[1,2]
```

示例 3：
```
输入：nums = [3,3], target = 6
输出：[0,1]
```

# 解法

## 法一：暴力解法

```cpp
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        int len = nums.size();
        for (int i = 0; i < len; i++) {
            for (int j = i + 1; j < len; j++) {
                if (nums[i] + nums[j] == target) {
                    return {i, j};
                }
            }
        }
        return {};
    }
```
算法复杂
时间复杂度：$\Omicron(n^2)$
空间复杂度：$\Omicron(1)$



## 法二：利用 hashtable 实现
将数组中的元素用hashtable存储，并在这个hashtable中查找 target 与数组中当前位置元素值的差值是否也在当前这个hashtable中，若存在，就将各自对应的下标值输出。
>注意：先查询哈希表中是否存在 target - nums[i]，然后再将 nums[i] 插入到哈希表中，保证不会让 nums[i] 和得到的差值重复。

```cpp
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        unordered_map<int, int> mp;
        vector<int>result;
        for (int i = 0; i < nums.size(); i++) {
            // mp[nums[i]] = i;
            auto it = mp.find(target - nums[i]);
            if (it != mp.end()) {
                result = {{it->second, i}}; 
            }
            mp[nums[i]] = i; 
        }
        return result;
    }
};
```
算法复杂
时间复杂度：$\Omicron(n)$
空间复杂度：$\Omicron(n)$，主要为hashtable的开销。