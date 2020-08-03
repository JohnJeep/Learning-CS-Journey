/*
 * @Author: JohnJeep
 * @Date: 2020-08-02 12:56:38
 * @LastEditTime: 2020-08-03 15:35:27
 * @LastEditors: Please set LastEditors
 * @Description: 冒泡排序实现 
 *               冒泡排序：分为从前往后依次轮询排序和从后往前依次轮询排序两种实现方式。
 *               复杂度：n * n
 * 
 *               思想：相邻元素之间逐次进行排序，每一次循环找到最大值或最小值，
 *                     剔除找到的最大值或最小值，然后剩下的数再重复轮询查找，直到完成整个的排序。
 *                     注意：增加标志位并没有减少算法的复杂度，算法的复杂度是在程序执行最坏的情况下统计的
 *            
 * @FilePath: /BubbleSort.c
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

void bubbleSort(int* array, int len)
{
    printf("排序前：");;
    printArray(array, len);

    int exchange = 1;    // 增加标志位，减少数组中原先已经是排好序的元素遍历的次数
    for (int i = 0; (i < len) && exchange; i++)
    {
        exchange = 0;    // 数组已经排好序
        for (int j = len - 1; j > i; --j)
        {
            if (array[j] < array[j-1])
            {
                int temp = array[j-1];
                array[j-1] = array[j];
                array[j] = temp;
                exchange = 1;   // 只要执行这条语句，表示数组中剩余的元素是没有排好序的
            }
        }
    }
    
    printf("排序后：");
    printArray(array, len);
}

int main(int argc, char *argv[])
{
    int array[] = {9, 8, 3, 5, 6, 2, 10, 1};
    int length = sizeof(array) / sizeof(array[0]);    

    bubbleSort(array, length);
    
    return 0;
}