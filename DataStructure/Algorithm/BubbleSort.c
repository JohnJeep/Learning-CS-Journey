/*
 * @Author: JohnJeep
 * @Date: 2020-08-02 12:56:38
 * @LastEditTime: 2020-08-02 12:58:06
 * @LastEditors: Please set LastEditors
 * @Description: 冒泡排序实现
 * @FilePath: /BubbleSort.c
 */ 
#include <stdio.h>
#include <stdlib.h>

void bubbleSort(int* array, int len)
{

}

int main(int argc, char *argv[])
{
    int array[] = {9, 8, 3, 5, 6, 2, 10, 1};
    int length = sizeof(array) / sizeof(array[0]);    

    bubbleSort(array, length);
    
    return 0;
}