/*
 * @Author: JonJeep
 * @Date: 2020-08-12 10:33:24
 * @LastEditTime: 2021-05-26 22:13:30
 * @LastEditors: Please set LastEditors
 * @Description: 数组名+1和数组名的地址+1
 *               结论: ①数组名+1，是+数组元素大小的字节数；数组名的地址+1，是+整个数组大小的字节数
 *                     ②指针+1，是+指针类型对应字节数；指针的地址+1，是+8
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

int main()
{
    short array[] = {1, 2, 3, 4, 5};
    
    printf("sizeof(short): %d\n", sizeof(short));
    printf("sizeof(array): %d\n", sizeof(array));
    printf("sizeof(array)/sizeof(short): %d\n", sizeof(array)/sizeof(short));

    printf("array address: %d\n", array);
    printf("&array[1] address: %d\n", &array[1]);
    printf("array + 2 address: %d\n", array + 2);     // address + sizeof(short) * 2
    printf("&array + 2 address: %d\n", &array + 2);   // address + sizeof(array) * 2
    printf("\n");

    short *p = array;
    printf("sizeof(p): %d\n", sizeof(p));             // operating system physical address bytes(64bit---8, 32bite---4) 
    printf("sizeof(*p): %d\n", sizeof(*p));
    printf("p address: %d\n", p);
    printf("p + 1 address: %d\n", p + 1);             // address + sizeof(short) * 1
    printf("&p address: %d\n", &p);                   // p oneself address
    printf("&p + 1 address: %d\n", &p + 1);           // address + 8

   return 0;
}

