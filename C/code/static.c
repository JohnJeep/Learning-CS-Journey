/*
 * @Author: JohnJeep
 * @Date: 2019-08-20 20:59:41
 * @LastEditTime: 2021-04-06 15:50:08
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <stdio.h>
#include <string.h>

int sub()
{
    static int count = 10;   // count变量在内存中位于全局区域，直到程序结束时，该变量才会被释放
    // int count = 10;       // count变量申请的内存空间位于栈区，当前函数体执行完成后，变量就被释放了
    return count--;
}


int main(int argc, char *argv[])
{
    for (int i = 0; i < 10; i++) {
        printf("%d\n",sub()); 
    }
    return 0;
}

