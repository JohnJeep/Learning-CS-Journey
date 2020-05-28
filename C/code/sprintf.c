/*
 * @Author: JohnJeep
 * @Date: 2020-05-28 15:59:55
 * @LastEditTime: 2020-05-28 16:18:06
 * @LastEditors: Please set LastEditors
 * @Description: sprintf函数功能：将输出发送到缓冲区,返回值是写入的字符数量
 * @reference: https://blog.csdn.net/nopoppy/article/details/52589745
 */ 
#include <stdio.h>
#include <string.h>
#include <malloc.h>

int main()
{
    char buf[32];
    char *str = "file to buf";

    int ret = sprintf(buf, "%s", str);
    printf("字符串个数: %d\n", ret);
    printf("字符串: %s\n", buf);

   return 0;
}