<!--
 * @Author: JohnJeep
 * @Date: 2021-04-09 11:29:58
 * @LastEditTime: 2021-04-09 16:14:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
-->
# 题目：数组中的第K个最大元素

描述

在未排序的数组中找到第 k 个最大的元素。请注意，你需要找的是数组排序后的第 k 个最大的元素，而不是第 k 个不同的元素。

示例 1:
```
输入: [3,2,1,5,6,4] 和 k = 2
输出: 5
```


示例 2:
```
输入: [3,2,3,1,2,4,5,5,6] 和 k = 4
输出: 4
```
说明:

你可以假设 k 总是有效的，且 1 ≤ k ≤ 数组的长度。

# 思路
建立一个大根堆，做 k - 1 次删除操作后堆顶元素就是我们要找的答案。

法一：K数值比较小时，可采用快速排序；复杂度：n * log(length)

法二：
1、当原数组中的数据顺序不可修改，且K数值很大时（海量数据），采用快速排序可能会占满内存，效率不高。因此使用最大堆数据结构实现。

2、先把前 k 个数据建立一个大根堆，然后对后面的 length-k 个数依次遍历，如果当前数值小于堆顶的值时，，则替换堆顶的值，然后对大根堆做向下调整。复杂度：n * log(k)

复杂度分析

- 时间复杂度：`O(n*log n)`，建堆的时间代价是 `O(n)`，删除的总代价是 `O(k*log n)`，因为 `k < n`，故渐进时间复杂为 `O(n + k *log n) = O(n *log n)`。
- 空间复杂度：`O(log n)`，即递归使用栈空间的空间代价。


关键点

建堆

调整堆

删除

# 代码实现
```cpp
// 调整堆
void adjustHeap(vector<int>& input, int i, int length) 
{
    int left = 2 * i + 1;  // i节点的左孩子，i从 0 开始
    int right = 2 * i + 2; // i节点的右孩子
    int max = i;           // 先设置父节点和子节点三个节点中最大值的位置为父节点下标
    if (left < length && input[left] > input[max])
    {
        max = left;  // 如果左孩子存在且大于最大值，更新最大值索引
    }
    if (right < length && input[right] > input[max]) 
    {
        max = right;  //如果右孩子存在且大于最大值，更新最大值索引
    }
    if (max != i) //最大值不是父节点，则进行交换
    {
        // int temp = input[i];
        // input[i] = input[max];
        // input[max] = temp;
        swap(input[i], input[max]);     // 交换对应索引位置的节点值
        adjustHeap(input, max, length); // 从最大值索引位置向下进行递归调用，保证子树也是最大堆
    }
}

int findKthLargest(vector<int>& nums, int k) {
    // 只需要建立一次最大堆，后面再调整最大堆
    for (int i =  nums.size() / 2; i >= 0; i--) {  
        adjustHeap(nums, i,  nums.size());  // 从堆树第一个非叶子节点开始调整
    }

    int heapSize = nums.size();
    for (int i = nums.size() - 1; i >= nums.size() - k + 1; i--) {
        swap(nums[0], nums[i]);  //将末尾结点补充到堆顶进行调整
        --heapSize;
        adjustHeap(nums, 0, heapSize);
    }
    return nums[0];
}
```