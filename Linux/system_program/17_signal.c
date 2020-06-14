/*
 * @Author: JohnJeep
 * @Date: 2020-06-13 09:06:11
 * @LastEditTime: 2020-06-13 10:05:24
 * @LastEditors: Please set LastEditors
 * @Description: 使用signal()函数捕捉信号
 * @FilePath: /system_program/17_signal.c
 */ 

#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

typedef __sighandler_t sighandler_t;

void show(int signo)
{
    printf("signal function catch signal: %d\n", signo);

}

int main(int argc, char *argv[])
{
    sighandler_t ret;
    int len;
    int flag[] = {SIGQUIT, SIGTSTP};

    len = sizeof(flag)/sizeof(int);

    for (int i = 0; i < len; i++)
    {
        ret = signal(flag[i], show);
    }
    
    if (ret == SIG_ERR)
    {
        perror("signal error.\n");
        exit(EXIT_FAILURE);
    }
    while(1);

    return 0;
}

