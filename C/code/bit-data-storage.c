/*
 * @Author: JohnJeep
 * @Date: 2019-12-13 08:51:58
 * @LastEditTime: 2019-12-13 10:09:36
 * @LastEditors: Please set LastEditors
 * @Description: 16位数据分为高8位和低8位，先写低字节，再写高字节
 *               参考：https://blog.csdn.net/mish84/article/details/51690526
 */
#include "stdio.h"

int main()
{
    printf("%d\n", sizeof(short));

    short originData = 0x1234;
    char H_bit = (originData >> 8) & 0xFF;     // 高8位
    char L_bit = originData & 0xFF;            // 低8位
    short data = (H_bit << 8) | L_bit;
    printf("%x\n", H_bit);
    printf("%x\n", L_bit);
    printf("%x\n", data);

    getchar();
    return 0;
}


