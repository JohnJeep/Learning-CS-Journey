/*
 * @Description: 函数指针的理解
 * @Author: JohnJeep
 * @Date: 2019-09-06 08:44:12
 * @LastEditTime: 2020-06-11 09:38:05
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"

int add(int x, int y)
{
    return x + y;
}

int sub(int x, int y)
{
    return x -y;
}

int multiple(int x, int y, int z)
{
    return (x * y * z);
}


// 定义函数指针 
int (*func)(int, int);

// 函数指针做函数参数
int funcPointerParam(int (*funcPointer)(int, int, int))  
{
    int val = funcPointer(3, 4, 5);  // 直接调用定义的函数指针
    printf("multi val: %d\n", val);
    return val;
}


int main()
{
    func = add;   //func指向函数的地址
    printf("add 函数地址: 0x%p\n", &add);
    printf("add 指向的数据: %x\n", add);

    printf("func指针的地址: %x\n", &func);
    printf("func指向函数add的地址: %x\n", func);
    int a = func(1, 3);
    printf("carry add operation: %d\n", a);


    func = &sub;
    printf("sub address: %x\n", &sub);
    printf("func指针的地址: %x\n", &func);
    printf("func指向函数sub的地址: %x\n", func);
    int b = func(4, 1);
    printf("carry add operation: %d\n", b);

   // 调用函数指针做函数参数
   funcPointerParam(multiple);   // multiple 指向函数的入口地址

    getchar();
    return 0;
}
