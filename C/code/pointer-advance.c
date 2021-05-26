/*
 * @Author: JohnJeep
 * @Date: 2019-12-28 14:28:20
 * @LastEditTime: 2021-05-26 22:23:50
 * @LastEditors: Please set LastEditors
 * @Description: C language pointer advance.
 */
#include <stdio.h>

int main()
{
    // 指针变量与指针指向的内存块测试
    char *p = NULL;
    printf("p address is: 0x%x \n", p);

    char array[] = "student";
    p = array;
    printf("p 指向array的地址: 0x%x \n", p);
    printf("array自身的地址: 0x%x \n", &array);

    p = p + 1;   // 改变指针指向的地址
    printf("改变p指向array的地址: 0x%x \n", p);
    printf("改变p指向array的地址后，指针变量p自身的值: %c \n", *p);
    printf("array的数据值: %s \n", array);

    // 改变指针指向的内存的值，不会影响指针本身的值
    array[2] = 'A';
    char *p1 = NULL;
    p1 = array; 
    printf("改变内存值后，p1 指向array的地址: 0x%x \n", p1);
    printf("改变内存值后，p1 指向array的值: %s \n", p1);


    /*   修改指针指向的内存出错，*buf定义的变量在内存的数据区
    char *buf = "abcdef";
    buf[1] = 'x';                             // 字符使用单引号   
    */
    char buf[] = "abcdef";
    buf[1] = 'x';                             // 字符使用单引号
    for (int i = 0; i < sizeof(buf); i++)
    {
        printf("%c \n", buf[i]);              // 字符串结尾有\0
    }
    getchar();
    return 0;
}