/*
 * @Author: JohnJeep
 * @Date: 2019-12-02 21:13:07
 * @LastEditTime: 2021-05-26 22:29:42
 * @LastEditors: Please set LastEditors
 * @Description: 关于指针的操作
 */

#include "stdio.h"

int main()
{
    int a = 10;
    int *p = NULL;
    
    p = &a;
    *p = 25;         // *在=的左边，给内存赋值，往内存写数据
    int b = *p;      // *在=的右边，取内存的值，从内存中读数据

    printf("b is value:%d \n", b); 
    printf("a is value:%d \n", a); 
    printf("p is value:%d \n", *p); 

    return 0;
}