/*
 * @Author: JohnJeep
 * @Date: 2020-08-03 15:35:55
 * @LastEditTime: 2020-08-03 16:07:57
 * @LastEditors: Please set LastEditors
 * @Description: 希尔排序算法实现
 *               思想：在插入算法的基础上进行数据的分组后实现的。每次分组减少 3 的倍数
 *               注意：希尔算法不稳定，是第一个将复杂度 n*n 降低至以下。              
 * 
 * @FilePath: /ShellSort.c
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

void shellSort(int* array, int len)
{
    printf("排序前：");
    printArray(array, len);

    int interval = len;
    do
    {
        interval = interval / 3 + 1;      // 通过大量的实现得出的规律

        for (int i = interval; i < len; i += interval)
        {
            int k = i;
            int temp = array[k];
            for (int j = i - interval; (j >= 0)&&(array[j] > temp) ; j -= interval)
            {
                array[j + interval] = array[j];
                k = j;
            }
            array[k] = temp;
        }
        
    } while (interval > 1);
        
    printf("排序后：");
    printArray(array, len);
}

int main(int argc, char *argv[])
{
    int data[] = {38, 65, 49, 97, 76, 13, 27};
    int length = sizeof(data) / sizeof(data[0]);

    shellSort(data, length);

   return 0;
}

