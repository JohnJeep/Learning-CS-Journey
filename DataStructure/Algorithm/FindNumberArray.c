/*
 * @Author: JohnJeep
 * @Date: 2020-07-13 21:23:24
 * @LastEditTime: 2021-03-07 22:24:36
 * @LastEditors: Please set LastEditors
 * @Description: 在1-100中某些数字组成的数组中，每个数字出现的次数可能是零次或多次，计算出现次数最多的数字。
 */ 
#include <stdio.h>
#include <stdlib.h>

#define LENGTH    100

void search(int data[], int len)
{
    int space[100] = {0};  // 定义一个数组存储重复出现的次数
    int max = 0;
    int index = 0;

    for (int i = 0; i < len; i++)
    {
        index = data[i] - 1;
        space[index]++;
    }
    
    for (int i = 0; i < LENGTH; i++)  // 求出数组中存储重复出现的最大数
    {
        if (max < space[i])
        {
            max = space[i];
        }
    }

    for (int i = 0; i < LENGTH; i++)
    {
        if (max == space[i])
        {
            printf("%d\n", i + 1);
        }
    }
}

int main(int argc, char *argv[])
{
    int array[] = {1, 3, 3, 4, 6, 3, 5, 6, 99, 6, 23, 44, 55, 6};   
    int len = sizeof(array)/sizeof(int);
    
    search(array, len);
    
    return 0;
}