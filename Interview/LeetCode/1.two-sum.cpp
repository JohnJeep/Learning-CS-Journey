/*
 * @Author: JohnJeep
 * @Date: 2021-03-20 00:03:46
 * @LastEditTime: 2021-04-04 18:16:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
/*
 * @lc app=leetcode id=1 lang=cpp
 *
 * [1] Two Sum
 */

// @lc code=start
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        // int left = 0;
        // int right = nums.size() - 1;
        // std::sort(nums.begin(), nums.end());
        // while (left < right) {
        //     int sum = nums[left] + nums[right];
        //     if (sum == target) {
        //         break;
        //     }
        //     if (sum < target) {
        //         left++;
        //     }
        //     else {
        //         right--;
        //     }            
        // }
        // return vector<int>{left, right};

        // 法一：暴力解法
        // int len = nums.size();
        // for (int i = 0; i < len; i++) {
        //     for (int j = i + 1; j < len; j++) {
        //         if (nums[i] + nums[j] == target) {
        //             return {i, j};
        //         }
        //     }
        // }
        // return {};

        // hash table
        unordered_map<int, int> hashtable;
        for (int i = 0; i < nums.size(); ++i) {
            auto it = hashtable.find(target - nums[i]);
            if (it != hashtable.end()) {
                return {it->second, i};
            }
            hashtable[nums[i]] = i;
        }
        return {};
    }
};
// @lc code=end

