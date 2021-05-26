/*
 * @Description: switch case语句简单实现
 * @Author: JohnJeep
 * @Date: 2019-08-25 12:08:51
 * @LastEditTime: 2021-05-26 22:22:12
 * @LastEditors: Please set LastEditors
 */
#include "stdio.h"

int main()
{
    int num;
    printf("请输入一个数：");
    scanf("%d", &num);

    if(num > 0) {  // 非零值包括正数和负数
        for(int i = 1; i <= num;)
        {
            switch (num) {
            case 1:
                printf("数字1! \n");
                continue;      // 会进入一个死循环
            case 2:
                printf("数字2! \n");
                break;        // 跳出循环
            case 3:
                printf("数字3! \n");
                break;
            default:
                printf("其他数字！ \n");
                break;
            }

        printf("请继续输入一个数：");
        scanf("%d", &num);
        }
    }
    else {
        printf("出错\n");
    }

    return 0;
}











