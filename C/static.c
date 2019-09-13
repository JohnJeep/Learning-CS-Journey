/*
 * @Description: 简单static的用法
 * @Author: JohnJeep
 * @Date: 2019-08-20 20:59:41
 * @LastEditTime: 2019-09-13 11:13:49
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"

int getOne()
{
    int t = 0;
    t++;
    printf("t的值为：%d \n", t);
    return t;
}

int getTwo()
{
    static int i = 0;
    i++;
    printf("i的值为：%d \n", i);
    return i;
}


int main() 
{
    int j =0;
    while ( j < 10)
    {
        getOne();
        getTwo();
        printf("\n");
        j++;
    }
    
    getchar();
    return 0;
}











