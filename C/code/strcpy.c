/*
 * @Author: JohnJeep
 * @Date: 2019-09-05 15:44:59
 * @LastEditTime: 2021-05-26 22:33:52
 * @LastEditors: Please set LastEditors
 * @Description: strcpy函数与strncpy函数
 *               source和destinin所指内存区域不可以重叠且destinin必须有足够的空间来容纳source的字符长度+'\0'
*                strcpy只是复制字符串，但不限制复制的数量，很容易造成缓冲溢出。strncpy要安全一些。
*                strncpy能够选择一段字符输出，strcpy复制全部的字符串。
 */
#include "stdio.h"
#include "string.h"

int main()
{
    char source[] = "hello word", destin[20] = {0};
    strncpy(destin, source, 8); 
    printf("use strncpy copy after:%s \n", destin);

    char source1[] = "hello word", destin1[20] = {0};
    strcpy(destin1, source1);
    printf("use strcpy copy after: %s\n", destin1);

    return 0;
}



