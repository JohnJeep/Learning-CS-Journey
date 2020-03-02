/*
 * @Author: your name
 * @Date: 2020-01-17 15:10:53
 * @LastEditTime: 2020-03-02 11:49:16
 * @LastEditors: Please set LastEditors
 * @Description: 结构体与数组
 * @FilePath: \C\struct_array.c
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

#define N 3
typedef struct T
{
    int age;
    char name[10];   // 用数组的方式存放字符串，且字符的长度为 10
}stu, *p;



int main()
{
    // 静态定义一个结构体数组
    stu A[2] = {
            {24, "jack"},
            {45, "John"},
    };     
    stu B[2] = {11, "first", 45, "second"};

    // A.age = 24;
    // A.name[0] = "John";
    // A.name[1] = "jack";
    for (int i = 0; i < 2; i++)
    {
        printf("%d %s \n", A[i].age, A[i].name);

    }

    // 动态定义结构体数组
    p pArray = (p *)malloc(N * sizeof(p));

    if(pArray != NULL)
    {
        free(pArray);
        pArray = NULL;
    }

   getchar();
   return 0;
}
