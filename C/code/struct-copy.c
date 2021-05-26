/*
 * @Author: JohNJeep
 * @Date: 2020-03-02 09:57:05
 * @LastEditTime: 2021-05-26 22:35:24
 * @LastEditors: Please set LastEditors
 * @Description: 关于结构体的相互赋值问题
 */

#include <stdio.h>

typedef struct struct_copy
{
    int age;
    char name[20];
}Stu;

Stu Wang = {24, "王明星"};
Stu Zhao = {25, "赵虎"};

void copyFunc(int a, int b)
{
    int temp;
    temp = a;
    a = b;
    b = temp;
}

void copyFuncPointer(int *a, int *b)
{
    int temp ;
    temp = *a;
    *a = *b;
    *b = temp;
}

void copyStruct(Stu *A, Stu *B)
{
    // 指针传递：通过地址传递
    // 通过形参去改变实参的值，一般采用地址传递
    // 按值传递不会改变实参的值，一般会有返回值
    // 按引用传递，适用于大型数组的传递，节省内存，可以改变实参的值

    *A = *B;
    //printf("age: %d, name: %s \n", A.age, A.name);
}

int main()
{
    int x = 12, y = 14; 
    copyFunc(x, y);                     // 并没有改变实参x和y的值
    printf("x: %d, y: %d \n", x, y);       

    int x1 = 12, y1 = 14; 
    copyFuncPointer(&x1, &y1);           // 按照指针传递，改变实参的值
    printf("x1: %d, y1: %d \n", x1, y1);  

    Stu Dest;
    copyStruct(&Dest, &Zhao);
    printf("age: %d, name: %s \n", Dest.age, Dest.name);

    return 0;
}




