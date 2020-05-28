/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-25 13:41:47
 * @LastEditTime: 2020-05-28 15:54:28
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"
#define  address  *(short*)0x040

int main()
{
    short a = 10;
    short *b;
    b = &a;
    printf("%d\n", a);
    printf("%x\n", b);
    printf("%d\n", *b);

    int c[5];
    c[0] = 1;
    c[1] = 3;
    int *d = &c[0];
    int *e;
    e = &c[1];

    unsigned char temp = 30;
    printf("%d\n", temp);

   // printf("%d\r", *d);
    printf("%d\n", d);
    printf("%x\n", &d);
    printf("%x\n", *&d);
    printf("%x\n", &c[0]);

    //printf("%d\r", *e);
    printf("%x\n", &c[1]);
    printf("%x\n", e);
    printf("%x\n", &e);
    printf("%x\n", *e);

    typedef struct tag_check_info 
    {
        char   x;
        short    crc16;
        short    sig;
        int    rsvd;
    } check_info;
    printf("%d\n", sizeof(check_info));



    getchar();
    return 0;
}






