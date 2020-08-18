/*
 * @Author: JohnJeep
 * @Date: 2020-08-18 09:58:01
 * @LastEditTime: 2020-08-18 22:02:44
 * @LastEditors: Please set LastEditors
 * @Description: 题目 求一个序列中的最小的K个数
 *               描述：输入n个整数，找出其中最小的K个数。例如输入4,5,1,6,2,7,3,8这8个数字，则最小的4个数字是1,2,3,4。
 *      
 *              思路：
 *                   法一：K数值比较小时，可采用快速排序；复杂度：n * log(length)
 *                   法二：1、当原数组中的数据顺序不可修改，且K数值很大时（海量数据），采用快速排序可能会占满内存，效率不高。因此使用最大堆数据结构实现
 *                         2、先把前 k 个数据建立一个大根堆，然后对后面的 length-k 个数一次遍历，
 *                            如果当前数值小于堆顶的值时，，则替换堆顶的值，然后对大根堆做向下调整。
 *    
 *                         复杂度：n * log(k)
 *
 * 
 */
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <vector>

using namespace std;

/**
 * @description: 找到序列中最大的一个数
 *               复杂度：n
 * @param {type} 
 * @return {type} 
 */
int findMax(int *array, int len)
{
    int max = 0;

    for (int i = 0; i < len; i++)
    {
        if (array[i] > max)
        {
            max = array[i];
        }
    }

    return max;
}

void printSortArray(int *array, int len)
{
    printf("after sort number: ");
    for (int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
}

/**
 * @description: 求一个序列中的最小的K个数
 *               
 * @param {type} 
 * @return {type} 
 */
int partition(int *array, int low, int high)
{
    int piv = array[low];
    while (low < high)
    {
        while ((low < high) && (array[high] >= piv))
        {
            high--;
        }
        array[low] = array[high];

        while ((low < high) && (array[low] <= piv))
        {
            low++;
        }
        array[high] = array[low];
    }
    array[low] = piv;

    return low;
}

void qsort(int *array, int len, int low, int high)
{
    if (low < high)
    {
        int pivot = partition(array, low, high);
        qsort(array, len, 0, pivot - 1);
        qsort(array, len, pivot + 1, high);
    }
}

/**
 * @description: 最大堆实现，调用用algorithm里的make_heap
 * @param {type} 
 * @return {type} 
 */
vector<int> GetLeastNumbers_Solution1(vector<int> input, int k)
{
    if (input.empty() || k == 0 || k > input.size())
    {
        return vector<int>{};
    }

    vector<int> result(input.begin(), input.begin() + k);

    make_heap(result.begin(), result.end()); //建立最大堆
    for (int i = k; i < input.size(); i++)
    {
        if (result[0] > input[i])
        {
            pop_heap(result.begin(), result.end()); //把最大值移到最后一个元素
            result.pop_back();                      //移除最后一个元素
            result.push_back(input[i]);
            push_heap(result.begin(), result.end()); //重新选出最大值
        }
    }
    sort_heap(result.begin(), result.end());

    return result;
}

/**
 * @description: 最大堆实现，没有用algorithm里的make_heap，自己实现heap
 * @param {type} 
 * @return {type} 
 */
void adjustHeap(vector<int>& input, int i, int length) 
{
    int left = 2 * i + 1;  // i节点的左孩子，i从 0 开始
    int right = 2 * i + 2; // i节点的右孩子
    int max = i;           // 先设置父节点和子节点三个节点中最大值的位置为父节点下标
    if (left < length && input[left] > input[max])
    {
        max = left;
    }
    if (right < length && input[right] > input[max])
    {
        max = right;
    }
    if (max != i) //最大值不是父节点，则进行交换
    {
        int temp;
        temp = input[i];
        input[i] = input[max];
        input[max] = temp;
        adjustHeap(input, max, length); //递归调用，保证子树也是最大堆
    }
}

vector<int> GetLeastNumbers_Solution2(vector<int> input, int k)
{
    if (input.empty() || k == 0 || k > input.size())
    {
        return vector<int>{};
    }

    vector<int> result;
    for (int i = input.size() / 2 - 1; i >= 0; i--) //建立堆
    {

        adjustHeap(input, i, k); //堆的大小为k
    }
    for (int i = k; i < input.size(); i++) //将后面的数依次和K个数的最大值比较
    {
        if (input[0] > input[i])
        {
            int temp = input[i];
            input[i] = input[0];
            input[0] = temp;
            adjustHeap(input, 0, k);
        }
    }
    for (int i = 0; i < k; i++)
    {
        result.push_back(input[i]);
    }
    return result;
}

// 测试用例01
void test01(int *array, int len, int low, int high, int k)
{
    qsort(array, len, low, high);
    printSortArray(array, len);
    if (k <= len)
    {
        printf("\nTop K num: ");
        for (int i = 0; i < k; i++)
        {
            printf("%d ", array[i]);
        }
    }
}

// 测试用例02
void test02(int *array, int len, int k)
{
    vector<int> v;

    for (int i = 0; i < len; i++)
    {
        v.push_back(*(array + i));
    }
    vector<int> result = GetLeastNumbers_Solution1(v, k);
    
    for (vector<int>::iterator it = result.begin(); it != result.end(); it++)
    {
        cout << *it << " ";
    }
}

// 测试用例03
void test03(int *array, int len, int k)
{
    vector<int> v;

    for (int i = 0; i < len; i++)
    {
        v.push_back(array[i]);
    }
    vector<int> result = GetLeastNumbers_Solution2(v, k);
    
    for (vector<int>::iterator it = result.begin(); it != result.end(); it++)
    {
        cout << *it << " ";
    }
}

int main(int argc, char *argv[])
{
    int data[] = {4, 5, 1, 6, 2, 7, 3, 8} ;
    // int data[] = {38, 65, 49, 97, 76, 13, 27};

    int length = sizeof(data) / sizeof(data[0]);

    int fmax = findMax(data, length);
    cout << "max num: " << fmax << endl;

    int k = 4;
    // test01(data, length, 0, length - 1, K);
    // test02(data, length, k);
    test03(data, length, k);

    return 0;
}

