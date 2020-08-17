/*
 * @Author: JohnJeep
 * @Date: 2019-09-25 13:41:47
 * @LastEditTime: 2020-08-17 11:10:36
 * @Description: C语言中指针取值与地址之间的联系
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"
#define  address  *(short*)0x040

int main()
{
    short a = 10;
    short *b;
    b = &a;
    printf("%d\n", a);
    printf("%x\n", b);
    printf("%d\n", *b);

    int c[5];
    c[0] = 1;
    c[1] = 3;
    int *d = &c[0];
    int *e;
    e = &c[1];

    unsigned char temp = 30;
    printf("%d\n", temp);

   // printf("%d\r", *d);
    printf("%d\n", d);
    printf("%x\n", &d);
    printf("%x\n", *&d);
    printf("%x\n", &c[0]);

    //printf("%d\r", *e);
    printf("%x\n", &c[1]);
    printf("%x\n", e);
    printf("%x\n", &e);
    printf("%x\n", *e);
    
    return 0;
}