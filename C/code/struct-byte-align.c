/*
 * @Author: JohnJeep
 * @Date: 2020-03-02 15:38:07
 * @LastEditTime: 2021-05-26 22:23:24
 * @LastEditors: Please set LastEditors
 * @Description: 结构体中字节对齐
 */
#include <malloc.h>
#include <stdio.h>
#include <string.h>

// 字节对齐按照结构体成员中最长的成员对齐

#pragma pack() // 指定特定的字节对齐
struct
{
    int age;
    char name;
    short id;
} nurse;

struct
{
    char name;
    int age;
    short id;
} nurse_1;

int main()
{
    printf("%d\n", sizeof(nurse));
    printf("%d\n", sizeof(nurse_1));

    return 0;
}
