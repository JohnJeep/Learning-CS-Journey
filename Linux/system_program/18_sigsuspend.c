/*
 * @Author: JohnJeep
 * @Date: 2020-06-13 11:27:05
 * @LastEditTime: 2020-06-14 17:28:06
 * @LastEditors: Please set LastEditors
 * @Description: 使用sigsuspend()函数模拟锁的方式来解决多个进程之间导致时序竞争的问题
 * @FilePath: /system_program/18_sigsuspend.c
 */ 
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <sys/types.h>

void catch_sigalarm(int signo)
{
    printf("catch signo: %d\n", signo);
}

unsigned int mysleep(unsigned int seconds)
{
    int ret;
    struct sigaction newact, oldact;
    sigset_t newmask, oldmask, suspmask;

    // 1、给SIGALRM信号设置一个捕捉信号函数
    newact.sa_handler = catch_sigalarm;
    sigemptyset(&newact.sa_mask);
    newact.sa_flags = 0;
    ret = sigaction(SIGALRM, &newact, &oldact);
    if (ret == -1)
    {
        perror("sigaction error,\n");
        exit(EXIT_FAILURE);
    }

    // 2、设置阻塞信号集，阻塞SIGALRM信号
    sigemptyset(&newmask);
    sigaddset(&newmask, SIGALRM);
    sigprocmask(SIG_BLOCK, &newmask, &oldmask);

    // 3、定时 n秒，到时后产生SIGALRM信号
    alarm(seconds);    

    // 4、构建一个调用 sigsuspend函数的临时有效的阻塞信号集，在临时阻塞信号集里面解除SIGALRN信号的阻塞
    suspmask = oldmask;
    sigdelset(&suspmask, SIGALRM);


    // 5、采用临时阻塞信号集suspmask阻塞原有的信号集，当sigsuspend被信号唤醒返回时，恢复原来阻塞的信号
    sigsuspend(&suspmask);
    ret = alarm(0);    // 返回延时时间的余下时间，增加程序的健壮性

    // 6、恢复SIGALRM信号的处理动作
    sigaction(SIGALRM, &oldact, NULL);

    // 7、解除对SIGALRM信号的阻塞
    sigprocmask(SIG_SETMASK, &oldmask, NULL);

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
