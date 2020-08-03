/*
 * @Author: JohnJeep
 * @Date: 2020-08-03 15:35:55
 * @LastEditTime: 2020-08-03 20:44:17
 * @LastEditors: Please set LastEditors
 * @Description: 快速排序算法实现
 *               思想：1、将数据分成两部分，即二分法划分
 *                     2、找到一个以partition为基础的数，将数组中的数据划分为两部分，
 *                        小的在partition数的左边，大的在partition数的右边
 *               复杂度：n*log(n)
 *               注意：当初始序列是有序时，快速排序可能是不稳定的
 * 
 * 
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

int partition(int* array, int low, int high)
{
    int pivot = array[low];
    int temp;

    if (array != NULL)
    {
        while (low < high)
        {
            while ((low < high) && (array[high] >= pivot))
            {
                high--;
            }
            temp = array[low];
            array[low] = array[high];
            array[high] = temp;      
                  
            while ((low < high) && (array[low] <= pivot))
            {
                low++;              
            }
            temp = array[low];
            array[low] = array[high];
            array[high] = temp;  
        }
    }
    return low;
}

void quickSort(int* array, int len, int low, int high)
{
    // 快速排序递归函数设置初始值
    if (low < high)
    {
        int pivot = partition(array, low, high);
        quickSort(array, len, low, pivot - 1);
        quickSort(array, len, pivot + 1, high);    
    }
}

int main(int argc, char *argv[])
{
    int array[] = {38, 65, 49, 97, 76, 13, 27};
    int len = sizeof(array) / sizeof(array[0]);

    printf("排序前：");
    printArray(array, len);

    quickSort(array, len, 0, len - 1);
    
    printf("排序后：");
    printArray(array, len);

   return 0;
}

