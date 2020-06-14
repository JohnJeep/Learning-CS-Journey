/*
 * @Author: JohnJeep
 * @Date: 2020-06-10 21:19:40
 * @LastEditTime: 2020-06-10 21:58:13
 * @LastEditors: Please set LastEditors
 * @Description: setitimer()函数的使用
 * @FilePath: /system_program/16_setitimer.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>

// void (*handle)(int sig);
void show(int sig)
{
    printf("catch signal.\n");
}

int main(int argc, char *argv[])
{
    struct itimerval new_value, old_value;
    // handle = show;
    signal(SIGALRM, show);   // 进行信号的捕捉
    
    // it_interval: 两次定时之间的间隔
    // it_value: 定时的时长

    new_value.it_value.tv_sec = 5;    // 定时 5s 钟
    new_value.it_value.tv_usec = 0;   //  0us

    new_value.it_interval.tv_sec = 3; // 两次定时之间间隔 3s
    new_value.it_interval.tv_usec = 0;
    

    if (setitimer(ITIMER_REAL, &new_value, &old_value) == -1)
    {
        perror("setitimer error.\n");
        return -1;
    }

    while (1)
    {
        /* code */
    }
    
    return 0;
}

