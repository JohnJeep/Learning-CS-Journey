/*
 * @Author: JohnJeep
 * @Date: 2020-06-13 10:31:07
 * @LastEditTime: 2020-06-13 23:18:57
 * @LastEditors: Please set LastEditors
 * @Description: 采用pause()和alarm()模拟sleep()函数。
 *               注意：使用pause()函数可能在多个进程之间导致时序竞争的问题。
 * @FilePath: /system_program/18_pause.c
 */ 
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>

void catch_sigalarm(int signo)
{
    printf("catch signo: %d\n", signo);
}

unsigned int mysleep(unsigned int seconds)
{
    int ret;
    struct sigaction act, oldact;
    
    act.sa_handler = catch_sigalarm;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    ret = sigaction(SIGALRM, &act, NULL);
    if (ret == -1)
    {
        perror("sigaction error,\n");
        exit(EXIT_FAILURE);
    }

    alarm(seconds);    
    ret = pause();     // 进程挂起，等待信号到来
    if ((ret == -1) && (errno == EINTR))
    {
        printf("pause catch success and return.\n");
        // exit(EXIT_SUCCESS);
    }catch signo: 1增加程序的健壮性
    ret = sigaction(SIGALRM, &oldact, NULL);  // 恢复SIGARM原先的处理方式
    if (ret == -1)
    {
        perror("sigaction error,\n");
        exit(EXIT_FAILURE);
    }
    
    return ret;
}

int main(int argc, char *argv[])
{
    while(1)
    {
        mysleep(3);
        printf("excute pause delay.\n");
    }

    return 0;
}

