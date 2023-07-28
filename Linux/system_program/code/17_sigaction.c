/*
 * @Author: JohnJeep
 * @Date: 2020-06-11 22:20:20
 * @LastEditTime: 2020-06-13 09:36:03
 * @LastEditors: Please set LastEditors
 * @Description: sigaction() 注册一个信号捕捉函数
 * @FilePath: /system_program/17_sigaction.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <fcntl.h>

struct sigaction act;

void catch(int signo)
{
    // 防止信号被中断后，不能重启，需要设置 SA_RESTART 标志
    if (signo == SIGALRM)
    {
#ifdef SA_RESTART
        act.sa_flags |= SA_INTERRUPT;
#endif
    }
    else
    {
      act.sa_flags |= SA_RESTART;  
    }    
    printf("catch sigal %d\n", signo);
}

int main(int argc, char *argv[])
{
    int ret;

    act.sa_handler = catch;                   // 信号捕捉后的处理函数catch()
    sigemptyset(&act.sa_mask);                // 将信号集的状态位清零
    sigaddset(&act.sa_mask, SIGQUIT);         // 在信号处理函数执行期间，将SIGQUIT信号集加到进程的信号屏蔽字中 
//    sigaddset(&act.sa_mask, SIGINT);       
    act.sa_flags = 0;                         // 使用默认属性

    // 捕捉SIGTSTP产生的信号
    ret = sigaction(SIGTSTP, &act, NULL);      // NULL忽略旧的信号变量
    if (ret == -1)
    {
        perror("sigaction error.\n");
        exit(EXIT_FAILURE);
    }

    while(1);

    return 0;
}
