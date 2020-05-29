/*
 * @Author: your name
 * @Date: 2020-03-02 15:38:07
 * @LastEditTime: 2020-03-02 16:18:34
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \C\struct_byteAlignment.c
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

// 字节对齐按照结构体成员中最长的成员对齐

#pragma pack()     // 指定特定的字节对齐
struct 
{
    int age;
    char name;
    short id;
}nurse;

struct 
{
    char name;
    int age;
    short id;
}nurse_1;

int main()
{
    printf("%d\n", sizeof(nurse));
    printf("%d\n", sizeof(nurse_1));
    getchar();
    return 0;
}




