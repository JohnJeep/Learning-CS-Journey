/*
 * @Description: 函数指针的理解
 * @Author: JohnJeep
 * @Date: 2019-09-06 08:44:12
 * @LastEditTime: 2019-09-13 10:53:54
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"

int add(int x, int y);
int sub(int x, int y);
int multiple(int x, int y, int z);


/**
 * @description: 定义函数指针 
 * @param {type} 
 * @return: 
 */
int (*func)();

int main()
{
    func = add;   //func指向函数的地址
    printf("add 函数地址: %x\n", &add);
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

    func = multiple;
    printf("乘积结果：%d \n", func(2, 3, 4));




    getchar();
    return 0;
}

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

















