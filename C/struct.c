/*
 * @Description: 打印结构体的长度
 * @Author: your name
 * @Date: 2019-09-02 22:41:03
 * @LastEditTime: 2019-09-13 14:01:57
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"


int main()
{
    struct variable
    {
        int a;
        int b;
        char c;
    }data;
    
    printf("结构体长度：%d\n", sizeof(data));

    getchar();
    return 0;
}










