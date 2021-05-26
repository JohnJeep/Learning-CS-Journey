/*
 * @Author: JohnJeep
 * @Date: 2020-03-12 08:32:58
 * @LastEditTime: 2021-05-26 22:15:43
 * @LastEditors: Please set LastEditors
 * @Description: (数组)将一列字符数转化为数字
 */
#include <stdio.h>
#include <string.h>

int str2num(char a[])
{
    //	printf("%c ",a[0]);
    int sz = 0, num = 0, t = 0, i = 0;
    sz = strlen(a);
    for (; sz > i; i++) {
        t = a[i] - '0';
        //printf("%c\n",t);
        num = 10 * num + t;
    }
    return num;
}

int main()
{
    char str[10]; //限定输入的字符串最多为9位，否则会内存泄漏
    int num = 0;

    printf("Input a string of number:");
    scanf("%s", &str);

    num = str2num(str);
    printf("the number is:%d\n", num);

    return 0;
}
