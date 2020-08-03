/*
 * @Author: JohnJeep
 * @Date: 2020-08-03 15:35:55
 * @LastEditTime: 2020-08-03 16:18:29
 * @LastEditors: Please set LastEditors
 * @Description: 快速排序算法实现
 *               思想：1、将数据分成两部分，即二分法划分
 *                     2、找到一个以partition为基础的数，将数组中的数据划分为两部分，
 *                        小的在partition数的左边，大的在partition数的右边
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

void quickSort(int* array, int len)
{
    printf("排序前：");
    printArray(array, len);


    


    printf("排序后：");
    printArray(array, len);
}

int main(int argc, char *argv[])
{
    int data[] = {38, 65, 49, 97, 76, 13, 27};
    int length = sizeof(data) / sizeof(data[0]);

    quickSort(data, length);

   return 0;
}

