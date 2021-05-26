/*
 * @Description: 变量在内存中分配的顺序
 * @Author: JohnJeep
 * @Date: 2019-09-06 09:40:47
 * @LastEditTime: 2021-05-26 22:27:21
 * @LastEditors: Please set LastEditors
 */

#include "stdio.h"


int main()
{
    int a = 1;
    int b = 2;

    struct test
    {
        int s;
        char t;
        short x;
    }var;     // 做了数据对齐和填充
    
    int m[10];
    int n[10];
    int arr[10];
    typedef char ptr[50];
    ptr text, data;

    printf("a is address: %x\n", &a);
    printf("value of a: %d\n", a);
    
    printf("b is address: %x\n", &b);
   //printf("b is address: %x\n", (size_t)&b);
    printf("value of b: %d\n", b);

    printf("var address of struct: %x\n", &var);
    printf("s address of struct: %x\n", &var.s);
    printf("t address of struct: %x\n", &var.t);
    printf("x address of struct: %x\n", &var.x);
    printf(" value of var: %d\n", sizeof(var));

    printf("m address of array: %x\n", &m);
    printf("n address of array: %x\n", &n);
    printf("arr address of array: %x\n", &arr);

    printf("n value of array: %d\n", sizeof(n));

    return 0;
}






