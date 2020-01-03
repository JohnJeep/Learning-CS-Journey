/*
 * @Author: your name
 * @Date: 2019-12-31 13:35:51
 * @LastEditTime : 2020-01-03 15:58:37
 * @LastEditors  : Please set LastEditors
 * @Description: 字符串提高
 *                1. C语言中，没有直接的字符串类型，用数组和指针表示字符串，以字符\0结尾
 *                2. 单个字符初始化使用单引号，多个字符串使用双引号；在单个字符中，字符\0与数字0等价
 *                3. 打印字符串使用 %s,  打印单个字符使用 %c
 *                4. 在函数中尽量不要直接输出形参，可能会产生指针指向的地址错误，使用中间量把形参接过来
 *                5. 操作形参指针之前，必须判断形参指针是否为NULL
 * @FilePath: \C\stringImprove.c
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void str_cpy(char *dest, char *src);
void str_cpy_1(char *dest, char *src);
void str_cpy_2(char *dest, char *src);
void str_cpy_3(char *dest, char *src);
int str_cpy_4(char *dest, char *src);

int main()
{
    // 定义数组时，没有指定数组长度，字符串最后没有\0结尾，则出现乱码
    char array[] = {'a', 'b', 'c', 'd'};
    printf("%s \n", array);

    // 定义数组时，设置数组长度，字符串最后默认为\0
    char buf[10] = {'a', 'b', 'c', 'd'};
    printf("%s \n", buf);

    char buf1[10] = "abcded";     // 常用字符串初始化
    printf("%s \n", buf1);    

    // sizeof: 得到数组的长度，包含结尾的\0
    // strlen: 得到字符串的长度，不包含结尾的\0
    char buf3[10] = "abcdef";
    printf("sizeof=%d, strlen=%d \n", sizeof(buf3), strlen(buf3));

    char buf4[10] = "abc0d";
    printf("%s \n", buf4);

    // 三种方法打印字符串中的字符
    char *p = NULL;
    p = buf3;
    for (int j = 0; j < strlen(buf3); j++)
    {
        // printf("buf3[%d]=%c \n", j, p[j]);
        printf("buf3[%d]=%c \n", j, *(p + j));
        //printf("buf3[%d]=%c \n", j, *(buf3 + j));
    }
    //注意：p++能使用，指针p指向的是数组buf3的首地址，改变的是指针指向的值，而不是数组buf3本身的值
    // buf3++不能使用，buf3为数组的首地址，编译器已经固定死了，不能修改，否则buf3的内存不能被回收
    

    // 使用自定义的str_cpy()函数
    char pDest[] = {0};
    char pSrc[] = "mingw64";

    str_cpy(pDest, pSrc);
    printf("pDest=%s \n", pDest);

    str_cpy_1(pDest, pSrc);
    printf("pDest_1=%s \n", pDest);

    str_cpy_2(pDest, pSrc);
    printf("pDest_2=%s \n", pDest);

    str_cpy_3(pDest, pSrc);
    printf("pDest_3=%s \n", pDest);

    int tmp;
    tmp = str_cpy_4(pDest, pSrc);
    if (tmp == 0)
    {
        printf("pDest_4=%s \n", pDest);
    }
    else
    {
        printf("program error! \n");
    }
    

    getchar();
    return 0;
}


//  封装一个strcoy()函数
void str_cpy(char *dest, char *src)
{
    int i = 0;
    for (i = 0; *(src+i) != '\0'; i++)
    {
        *(dest + i) = *(src + i);
    }
    *(dest + i) = '\0';
}

void str_cpy_1(char *dest, char *src)
{
    while (*src != '\0')
    {
        *dest = *src;    // src首地址的值赋给dest首地址的值
        dest++;
        src++;
    }
    *dest = '\0';       // 最后一个位在、设置为\0
}

void str_cpy_2(char *dest, char *src)
{
    while (*src != '\0')
    {
                            // * 与++的优先级在同一级，从右向左运算，但dest++是先运算后再执行其它运算。
        *dest++ = *src++;   // 先执行*dest = *src,再执行dest++，src++
    }
    *dest = '\0';
}

void str_cpy_3(char *dest, char *src)
{
    //while (*dest++ = *src++)  
    while ((*dest++ = *src++) != '\0')     
    {
        // 执行顺序：先执行*dest = *src,再执行dest++，src++
        // 判断*dest是否等于0，若等于0，则跳出循环。
        NULL;
    }
    *dest = '\0';
}

int str_cpy_4(char *dest, char *src)
{
    if((dest == NULL) | (src == NULL))
    {
        printf("pointer is null:error! \n");
        return -1;
    }
    while ((*dest++ = *src++) != '\0')     
    {
        NULL;
    }
    *dest = '\0';    
    return 0;
}