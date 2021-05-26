/*
 * @Author: John Jeep
 * @Date: 2019-08-29 22:32:43
 * @LastEditTime: 2021-05-26 22:29:04
 * @LastEditors: Please set LastEditors
 * @Description: 简单的指针函数例子
 */
#include "stdio.h"

int *complare(int *x, int *y)
{
    if(*x > *y) {
        printf("较大值: %x \n", x);
        return x;
    }
    else {
        printf("较小值的memory为: 0X=%x \n", y);
        return y;
    }
}

int main()
{
    int a = 5;
    int b = 10;
    int *p = NULL;

    p = complare(&a, &b);
    printf("%d \n", *p);

    return 0;
}
