/*
 * @Author: JohnJeep
 * @Date: 2019-12-13 10:36:49
 * @LastEditTime: 2019-12-13 11:56:31
 * @LastEditors: Please set LastEditors
 * @Description: 整型类的数据转化为字符串，显示为浮点型的
 * @FilePath: \C\intToString copy.c
 */

#include "stdio.h"
#include "string.h"
#define BUF_SIZE 16

unsigned char int_to_astr(int val, char *pbuf, unsigned char validBits);

int main()
{
    int data = -8765;
    char arr[10] = {0};

    int temp = int_to_astr(data, arr, 0);
    printf("%s \n", arr);
    printf("%d \n", temp);

    getchar();
    return 0;
}

/**
 * @description: 整型类的数据转化为字符串，显示为浮点型的 
 * @param: {int: 整型数; *pbuf: 指向数组的指针;  validBits: 保留小数点后几位} 
 * @return: cnt: 字符串的总长度，包括小数点(.)
 */
unsigned char int_to_astr(int val, char *pbuf, unsigned char validBits)
{
    char buf[BUF_SIZE];
    char *pstr;
    unsigned char len, cnt;
    unsigned char minus;

    memset(buf, 0, BUF_SIZE);
    minus = 0;
    if (val == 0)
    {
        pbuf[0] = '0';
        return 1;
    }
    else if (val < 0)
    {
        minus = 1;
        val = -val;
    }

    pstr = &buf[BUF_SIZE - 1];
    len = 0;
    // val的值存储在buf[]数组中
    do
    {
        //取num最低位 字符0~9的ASCII码是48~57；简单来说数字0+48=48，ASCII码对应字符'0'
        *pstr-- = (char)(48 + (val % 10)); // 从buf[BUF_SIZE-1]往前面依次存储数据val的余数.
        val /= 10;                           // 去掉最低位
        len++;                               // 得到输入数据的长度
    } while (val > 0);
    pstr++;
    cnt = 0;
    if (minus)
    {
        *pbuf++ = '-';
        cnt++;
    }
    if (len <= validBits) // 显示小于1的数
    {
        *pbuf++ = '0';
        *pbuf++ = '.';
        cnt += 2;
        while (validBits > len)
        {
            *pbuf++ = '0';
            cnt++;
            validBits--;
        }
        memcpy(pbuf, pstr, len); // 向pbuf数组中写入长度为len个的 pstr指向的值
        cnt += len;
        pbuf[len] = 0;
    }
    else
    {
        while (len > validBits) // 显示大于1的数
        {
            // 得到数据的整数部分
            *pbuf++ = *pstr++;
            cnt++;
            len--;
        }
        if(validBits == 0)
        {
            *pbuf = ' ';    // 下一位设置为空格
            memcpy(pbuf, pstr, validBits);
            cnt += validBits;
        }
        else
        {
            *pbuf++ = '.';          
            memcpy(pbuf, pstr, validBits);
            cnt += (validBits + 1);       
        }
        pbuf[validBits] = 0;         
    }
    return cnt;
}
