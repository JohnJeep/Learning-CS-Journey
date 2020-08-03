/*
 * @Author: JohnJeep
 * @Date: 2020-08-03 20:52:19
 * @LastEditTime: 2020-08-03 22:03:05
 * @LastEditors: Please set LastEditors
 * @Description: 归并排序算法实现
 * @FilePath: /MergeSort.c
 */
#include <stdio.h>
#include <stdlib.h>


void printArray(int* array, int len)
{
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");
}


void dealMerge(int* src, int* des, int low, int high, int mid)
{
    int s = low;
    int t = mid + 1;
    int k = low;

    while ((s <= mid) && (t <= high))
    {
        if (src[s] < src[t])
        {
            des[k++] = src[s++];
        }
        else
        {
            des[k++] = src[t++];
        }
    }
    while (s <= mid)    // 单个分组中还剩几个尾部元素
    {
        des[k++] = src[s++];
    }
    while (t <= high)
    {
        des[k++] = src[t++];
    }
} 

// 每次分为 2 组，当只剩下一个元素时就不在划分
void partition(int* src, int* des, int low, int high, int max)
{
    if (low == high)   // 当数组中只有一个元素时，则不需要划分
    {
        des[low] = src[low];
    }
    else
    {
        int mid = (low + high) / 2;
       int* space = (int*)malloc(sizeof(int) * max);

        if (space != NULL)
        {
            partition(src, space, low, mid, max);            // 递归调用
            partition(src, space, mid + 1, high, max);
            dealMerge(space, des, low, high, mid);
        }
        free(space);
    }
}

void mergeSort(int* array, int len)
{
    partition(array, array, 0, len - 1, len);
}

int main(int argc, char *argv[])
{
    int array[] = {38, 65, 49, 97, 76, 13, 27};
    int len = sizeof(array) / sizeof(array[0]); 
    
    printf("排序前：");
    printArray(array, len);
    
    mergeSort(array, len);

    printf("排序后：");
    printArray(array, len);

    return 0;
}
