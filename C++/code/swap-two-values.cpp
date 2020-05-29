/*
 * @Author: JohnJeep
 * @Date: 2020-05-29 14:11:08
 * @LastEditTime: 2020-05-29 23:27:46
 * @LastEditors: Please set LastEditors
 * @Description: 普通引用的实现
 */ 
#include <iostream>

void swap01(int a, int b)
{
    int c = 0;

    c = a;
    a = b;
    b = c;
    // printf("a=%d, b=%d", a, b);
}

void swap02(int *a, int *b)
{
    int c;

    c = *a;
    *a = *b;
    *b = c;
}

void swap03(int &a, int &b)   // 传引用
{
    int c = 0;

    c = a;
    a = b;
    b = c;
}
 int main()
 {
    int a = 11;
    int b = 22;
    int c = 33;
    int d = 44;
    int e = 55;
    int f = 66;


    swap01(a, b);
    printf("swap01: a=%d, b=%d\n", a, b);

    swap02(&c, &d);
    printf("swap02: c=%d, d=%d\n", c, d);

    swap03(e, f);
    printf("swap03: e=%d, f=%d\n", e, f);

    return 0;
 }












