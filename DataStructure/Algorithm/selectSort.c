/*
 * @Author: JohnJeep
 * @Date: 2019-12-03 20:52:08
 * @LastEditTime: 2020-08-17 22:27:07
 * @LastEditors: Please set LastEditors
 * @Description: 选择排序算法实现    
 *               无序序列逐渐减少，有序序列逐渐增多       
 * @FilePath: /SelectSort.c
 */
#include "stdio.h"

void selectSort(int* array, int len)
{
    printf("选择排序前：");
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");

    for (int i = 0; i < len-1; i++)
    {
        for (int j = i+1; j < len; j++)
        {
            if(array[i] > array[j])
            {
                int temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }
    
    printf("选择排序后：");
    for(int i = 0; i < len; i++)
    {
        printf("%d ", array[i]);
    }
    printf("\n");
}

int main(int argc, char *argv[])
{
    int array[] = {9, 8, 3, 5, 6, 2, 10, 1};
    int length = sizeof(array) / sizeof(array[0]);

    selectSort(array, length);

    return 0;
}