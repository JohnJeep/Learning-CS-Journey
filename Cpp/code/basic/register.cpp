/*
 * @Author: your name
 * @Date: 2020-05-29 09:06:50
 * @LastEditTime: 2020-05-29 10:09:39
 * @LastEditors: Please set LastEditors
 * @Description: register关键字：请求编译器让变量直接放在寄存器里面
 *               C语言中无法取得寄存器变量地址，在C++中，支持register
 *               关键字，C++编译器发现程序中需要取 register变量的地址时，
 *               register对变量的声明变得无效。
 */ 
// #include <stdio.h>
#include <iostream>
#include <sys/types.h>

int main()
{
    register int a = 10;
    printf("register a address: 0x%x\n", &a);

    system("pause");
    return 0;
}