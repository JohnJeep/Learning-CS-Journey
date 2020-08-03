/*
 * @Author: JohnJeep
 * @Date: 2020-08-02 12:30:03
 * @LastEditTime: 2020-08-03 15:50:42
 * @LastEditors: Please set LastEditors
 * @Description: 插入法排序实现
 *               思路：1、将需要插入的元素拿出来，留出空位
 *                     2、将拿出的元素与序列中在此元素之前的所有元素进行对比，符合条件的元素依次向后移动
 * @FilePath: /InsertSort.c
 */ 
#include <stdio.h>
#include <stdlib.h>

void insertSort(int* array, int len)
{
    printf("插入排序前：");
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");

    for (int i = 1; i < len; i++)
    {
        int k = i; 
        int temp = array[i];                // 需要插入的位置
        for (int j = i - 1; (j >= 0)&&(array[j] > temp); j--)   // 按照从小到大的顺序排序
        {
            array[j+1] = array[j]; // 元素后移
            k = j;                 // 插入的位置
        }
        array[k] = temp;           // 元素插入
    }

    printf("插入排序后：");
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");
}

int main(int argc, char *argv[])
{
    int data[] = {38, 65, 49, 97, 76, 13, 27};
    int length = sizeof(data) / sizeof(data[0]);
    
    insertSort(data, length);

    return 0;
}
