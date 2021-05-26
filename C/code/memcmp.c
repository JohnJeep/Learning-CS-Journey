/*
 * @Description: 单引号与双引号区别
 *               把存储区 str1 和存储区 str2 的前 n 个按照字节进行比较
 *               函数的返回值为一个整数
 * @Author: JohnJeep
 * @Date: 2019-09-12 10:56:05
 * @LastEditTime: 2021-05-26 22:24:49
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"
#include "string.h"

int main()
{
    char *a = "it's very good";
    char *b = "it's quite good";
    
    int val = memcmp(b, a, strlen(b));
    printf("%d \n", val);

    return 0;
}
