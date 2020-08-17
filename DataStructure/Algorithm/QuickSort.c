/*
 * @Author: JohnJeep
 * @Date: 2020-08-03 15:35:55
 * @LastEditTime: 2020-08-17 23:25:44
 * @LastEditors: Please set LastEditors
 * @Description: 快速排序算法实现
 *               思想：1、将数据分成两部分，即二分法划分
 *                     2、找到一个以partition为基础的数，将数组中的数据划分为两部分，
 *                        小的在partition数的左边，大的在partition数的右边
 *               复杂度：n*log(n)
 *               注意：当初始序列是有序时，快速排序可能是不稳定的
 * 
 *              快速排序优化：https://www.cnblogs.com/ttltry-air/archive/2012/08/06/2625512.html
 * @FilePath: /quickSort.c
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

void printArray(int* array, int len)
{
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");
}

void swap(int array[], int low, int high)
{
    int temp = array[low];
    array[low] = array[high];
    array[high] = temp; 
}

/**
 * @description:  // 使用两个索引low和high，分别从左右两端进行扫描，low扫描到大于等于pivot的元素就停止，high扫描到小于等于pivot的元素也停止，
 *                   交换两个元素，持续这个过程直到两个索引相遇，此时的pivot的位置就落在了low，然后交换pivot和low的位置
 * @param {type} 
 * @return {type}  low, high为索引下标
 */
int partition(int* array, int low, int high)
{
    int pivot = array[low];

    if (array != NULL)
    {
        while (low < high)
        {
            while ((low < high) && (array[high] >= pivot))
            {
                high--;
            }   
            array[low] = array[high];
            
            while ((low < high) && (array[low] <= pivot))
            {
                low++;              
            }
            array[high] = array[low]; 
        }
        array[low] = pivot;
    }
    return low;
}


// 取三个数的中位数
static int median(int arr[], int a, int b, int c) 
{
    return arr[a] < arr[b] ? (arr[b] < arr[c] ? b : arr[a] < arr[c] ? c : a)
            : arr[b] > arr[c] ? b : arr[a] > arr[c] ? c : a;
}


/**
 * @description: 选择排序优化：通过求三个数中的中位数来得到合适的pivot
 *               大于40的数组使用median-of-nine选择pivot，
 *               大小在7到40之间的数组使用three-of-median选择pivot,
 *               等于7的数组直接选择中数作为pivot，
 *               小于7的数组则直接使用插入排序
 * @param {type} 
 * @return {type} 
 */
static int partitionOptimize(int arr[] , int low, int high) 
{
    int len = high - low + 1;  // 计算数组长度
    
    // 在数组大小小于7的情况下使用插入排序
    if (len < 7) 
    {
        for (int i = low; i <= high; i++) 
        {
            for (int j = i; j > low && arr[j - 1] > arr[j]; j--) 
            {
                swap(arr, j, j - 1);
            }
        }
    }
      
    int med = low + (len >> 1); // 求出中点，大小等于7的数组选择pivot

    // 大小大于7
    if (len > 7) 
    {
        int l = low;
        int n = low + len - 1;
        if (len > 40)   // 大数组，采用median-of-nine选择
        { 
            int s = len / 8;
            l = median(arr, l, l + s, l + 2 * s); // 取样左端点3个数并得出中数
            med = median(arr, med - s, med, med + s); // 取样中点3个数并得出中数
            n = median(arr, n - 2 * s, n - s, n); // 取样右端点3个数并得出中数
        }
        med = median(arr, l, med, n); // 取中数中的中数
    }
    
    swap(arr, low, med);  // 交换pivot到左端点

    return partition(arr, low, high);
  }


void quickSort(int* array, int len, int low, int high)
{
    // 快速排序递归函数设置初始值
    if (low < high)
    {
#if 0 
        int pivot = partition(array, low, high);  // 第一次得到轴的大小
#endif
        int pivot = partitionOptimize(array, low, high);  // 调用优化后的快速排序，第一次得到轴的大小
        quickSort(array, len, low, pivot - 1);
        quickSort(array, len, pivot + 1, high);    
    }
}

// 打印快速排序
void test01(int *array, int len)
{
    printf("test case01\n");
    printf("排序前：");
    printArray(array, len);

    quickSort(array, len, 0, len - 1);
    
    printf("排序后：");
    printArray(array, len);
}

// 快速排序的优化
void test02(int *array, int len)
{
    printf("test case02\n");
    printf("排序前：");
    printArray(array, len);

    quickSort(array, len, 0, len - 1);
    
    printf("排序后：");
    printArray(array, len);
}


int main(int argc, char *argv[])
{
    // int data[] = {38, 65, 49, 97, 76, 13, 27};
    // int data[] = {38, 65, 49, 97, 76, 13};
    int data[] = {38, 65, 49, 97, 76, 13, 87, 25, 69};
    int length = sizeof(data) / sizeof(data[0]);

    // test01(data, length);
    test02(data, length);

   return 0;
}

