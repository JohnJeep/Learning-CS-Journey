/*
 * @Author: JohnJeep
 * @Date: 2020-05-29 14:11:08
 * @LastEditTime: 2020-05-29 16:16:39
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
    printf("a=%d, b=%d", a, b);
}

// void swap02(int *a, int *b)
// {
//     int c;
//     c = a;
//     a = b;
//     b = c;
//     printf("a=%d, b=%d", a, b);
// }

 int main()
 {
    int a = 11;
    int b = 22; 

    swap01(a, b);

    return 0;
 }












