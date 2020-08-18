/*
 * @Author: JohnJeep
 * @Date: 2020-08-18 20:22:11
 * @LastEditTime: 2020-08-18 21:07:53
 * @LastEditors: Please set LastEditors
 * @Description: 堆排序实现
 * @FilePath: /HeapSort.c
 */
#include <stdio.h>
#include <stdlib.h>

/**
 * @description: 建立最大堆，找处最大元素
 * @param {type} *array: s数组首地址
 * @param {type} len: 数组长度
 * @param {type} n: 元素在数组中的位置
 *             
 * @return {type} 
 */
void maxHeap(int *array, int len, int n)
{
    int left = 2 * n + 1;           // 数组的下标从零开始，当前结点的左子结点
    int right = 2 * n + 2;          // 当前结点的右子结点
    int max;

    if (left < len && array[left] > array[n])
    {
        max = left;
    }
    else
    {
        max = n;
    }

    if (right < len && array[right] > array[max])
    {
        max = right;
    }

    if (max != n)
    {
        int temp = array[n];
        array[n] = array[max];
        array[max] = temp;
        maxHeap(array, len, max);
    }
}

void heapSort(int *array, int len)
{
    int i;

    for (i = (len / 2) - 1; i >= 0; --i)
    {
        maxHeap(array, len, i);
    }
    for (i = len - 1; i >= 0; --i)  // 数组下标索引从零开始，最大为 len-1
    {
        int temp = array[0];
        array[0] = array[i];
        array[i] = temp;            // 第一次将第一个数与最后一个数相互交换
        maxHeap(array, i, 0);
    }
}


int main(int argc, char *argv[])
{
    int data[] = {49, 38, 65, 97, 76, 13, 27, 55, 04};
    int length = sizeof(data) / sizeof(data[0]);

    printf("before heap sort: ");
    for (int i = 0; i < length; i++)
    {
        printf("%d ", data[i]);
    }
    
    heapSort(data, length);
    
    printf("\nafter heap sort: ");
    for (int i = 0; i < length; i++)
    {
        printf("%d ", data[i]);
    }
    
    return 0;
}