/*
 * @Author: JohnJeep
 * @Date: 2020-06-11 22:59:46
 * @LastEditTime: 2020-06-11 23:27:47
 * @LastEditors: Please set LastEditors
 * @Description: 显示未决信号集合的状态
 * @FilePath: /system_program/17_pending_signal_set.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <fcntl.h>

void show(sigset_t *p)
{
    int i;

    for (i = 1; i < 32; i++)
    {
        if (sigismember(p, i) == 1)
        {
            putchar('1');
        }
        else
        {
            putchar('0');
        }
    }
    
    printf("\n");
}

int main(int argc, char *argv[])
{
    sigset_t new_set, old_set, ped;

    sigemptyset(&new_set);
    sigaddset(&new_set, SIGQUIT);  // Ctrl + '\'
    sigaddset(&new_set, SIGTSTP);  // Ctrl + z
    sigaddset(&new_set, SIGINT);   // Ctrl + C
    sigprocmask(SIG_BLOCK, &new_set, &old_set);

    while (1)
    {
        sigpending(&ped);
        show(&ped);
        sleep(1);
    }

    return 0;
}
