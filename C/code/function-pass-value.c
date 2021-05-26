/*
 * @Author: JohnJeep
 * @Date: 2020-01-15 13:59:14
 * @LastEditTime: 2021-05-26 22:16:30
 * @LastEditors: Please set LastEditors
 * @Description: 指针与函数传值：局部变量的值尽量不要传递到函数外边
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

int *fun ()
{
    int a = 10;  
    printf("局部变量 a 地址: 0x%x\n", &a); 
    return &a;

    // 采用下面的方式，间接给指针p赋值，给指针p有明确的指向内存空间的值
    // int *p = NULL; 
    // p = &a;
    // return p;
}

int *fun_1()
{   
    static int a = 20; 
    return &a;
}

int main()
{
    int *tmp = NULL;
    printf("tmp自身地址：0x%x\n", &tmp);
    tmp = fun();                          // 没有改变tmp自身的地址，tmp指向地址的
    printf("tmp地址: 0x%x\n", &tmp);
    //printf("tmp指向的值: %d\n", *tmp);  // 函数fun执行完后内存被释放，导致指针tmp指向的内存没有值，出现野指针，产生段错误

    tmp = fun_1();
    printf("tmp指向的值: %d\n", *tmp);   // static关键字存放的值在全局区，指针tmp指向的内存有对应的值
    printf("tmp地址: 0x%x\n", &tmp);

   return 0;
}