/*
 * @Author: JohnJeep
 * @Date: 2020-06-04 22:02:36
 * @LastEditTime: 2020-06-04 22:23:14
 * @LastEditors: Please set LastEditors
 * @Description: linux进程中子进程与父进程遵循的原则：读时共享写时复制
 * @FilePath: /system_program/07-share_process.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>

int var = 10;

int main(int argc, char *argv[])
{
    pid_t pd;

    printf("fork begin.");
    printf("var=%d\n", var);
    pd = fork();
    if (pd == -1)
    {
        perror("create fork error.\n");
        exit(1);
    }
    else if (pd == 0)
    {
        sleep(1);
        var = 50;   // 子进程作修改
        printf("I am child. getpid=%d, getppid=%d, var=%d\n", getpid(), getppid(), var);
    }
    else
    {
        sleep(1);
        // var = 100;
        printf("I am parent. getpid=%d, getppid=%d, var=%d\n", getpid(), getppid(), var);        
    }
    printf("var=%d\n", var);
    
    return 0;
}



