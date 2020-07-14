/*
 * @Author: JohnJeep
 * @Date: 2019-12-03 20:52:08
 * @LastEditTime: 2020-07-14 21:50:17
 * @LastEditors: Please set LastEditors
 * @Description: 选择排序算法实现           
 * @FilePath: /selectSort.c
 */
#include "stdio.h"

// 数组作为形参传入，等价于一个指针的的传入
// int buf[5];
// func(int a, buf);  ----------- func(int a, *p);

int main() 
{
    int a[] = {9, 8, 3, 5, 6, 2, 10, 1};
    int i, j, temp;


    int n = sizeof(a) / sizeof(a[0]);
    printf("选择排序前：");;
    for(i = 0; i < n; i++)
    {
        printf("%d ", a[i]);
    }
    printf("\n");

    for ( i = 0; i < n-1; i++)
    {
        for ( j = i+1; j < n; j++)
        {
            if(a[i] > a[j])
            {
                temp = a[i];
                a[i] = a[j];
                a[j] = temp;
            }
            
        }
        
    }
    
    printf("选择排序后：");;
    for(i = 0; i < n; i++)
    {
        printf("%d ", a[i]);
    }
    printf("\n");

    return 0;
}