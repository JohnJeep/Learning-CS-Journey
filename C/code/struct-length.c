/*
 * @Author: JohnJeep
 * @Date: 2019-09-02 22:41:03
 * @LastEditTime: 2020-08-17 11:12:20
 * @LastEditors: Please set LastEditors
 * @Description: 打印结构体的长度
 */
#include "stdio.h"

int main()
{
    struct variable
    {
        int a;
        int b;
        char c;
    }data;
    
    printf("结构体长度：%d\n", sizeof(data));

    typedef struct tag_check_info 
    {
        char     x;
        short    crc16;
        short    sig;
        int      rsvd;
    } check_info;
    printf("%d\n", sizeof(check_info));

    return 0;
}










