/*
 * @Author: JonJeep
 * @Date: 2020-08-17 10:57:56
 * @LastEditTime: 2021-05-23 17:52:05
 * @LastEditors: Please set LastEditors
 * @Description: 自定义实现字符串的拷贝函数
 */
#include <stdio.h>
#include <string.h>

//  封装一个strcopy()函数
void str_cpy_1(char *dest, const char *src)
{
    int i = 0;
    for (i = 0; *(src + i) != '\0'; i++) {
        *(dest + i) = *(src + i); // 等价于dest[i] = src[i];
    }
    *(dest + i) = '\0';
}

void str_cpy_2(char *dest, const char *src)
{
    while (*src != '\0') {
        *dest = *src; // src首地址的值赋给dest首地址的值
        dest++;
        src++;
    }
    *dest = '\0'; // 最后一位设置为\0
}

void str_cpy_3(char *dest, const char *src)
{
    while (*src != '\0') {
        // * 与++的优先级在同一级，从右向左运算，但dest++是先运算后再执行其它运算。
        *dest++ = *src++; // 先执行*dest = *src,再执行dest++，src++
    }
    *dest = '\0';
}

void str_cpy_4(char *dest, const char *src)
{
    //while (*dest++ = *src++)
    while ((*dest++ = *src++) != '\0') {
        // 执行顺序：先执行*dest = *src,再执行dest++，src++
        // 判断*dest是否等于0，若等于0，则跳出循环。
        ;
    }
    *dest = '\0';
}

int str_cpy_5(char *dest, const char *src)
{
    char *d = dest;
    const char *s = src;

    if ((d == NULL) | (s == NULL)) {
        printf("pointer is null:error! \n");
        return -1;
    }
    while ((*d++ = *s++) != '\0') {
        ;
    }
    *d = '\0';

    return 0;
}

int main()
{
    // 使用自定义的str_cpy()函数
    char src[] = "hello";
    char dest[] = {0};

    // str_cpy_1(dest, src);
    // printf("dest_1 =%s \n", dest);
    // printf("src_1 =%s \n", src);
    // printf("\n");

    // str_cpy_2(dest, src);
    // printf("dest_2=%s \n", dest);
    // printf("\n");

    // str_cpy_3(dest, src);
    // printf("dest_3=%s \n", dest);
    // printf("src=%s \n", src);
    // printf("\n");

    str_cpy_4(dest, src);
    printf("dest_4 =%s \n", dest);
    printf("src_4 =%s \n", src);
    printf("\n");

    int tmp = str_cpy_5(dest, src);
    if (tmp == 0) {
        printf("dest_5=%s \n", dest);
        printf("src_5 =%s \n", src);
        printf("\n");
    }
    else {
        printf("program error! \n");
    }

    return 0;
}