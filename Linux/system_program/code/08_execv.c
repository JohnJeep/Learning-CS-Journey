/*
 * @Author: JohnJeep
 * @Date: 2020-06-04 22:57:25
 * @LastEditTime: 2020-06-04 23:20:36
 * @LastEditors: Please set LastEditors
 * @Description: execv函数家族系列
 * @FilePath: /system_program/execv.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    pid_t pd;

    pd = fork();
    if (pd == -1)
    {
        perror("create fork error.\n");
        exit(1);
    }
    else if (pd == 0)
    {
        // execlp("ls", "ls", "-l", NULL);
        // execl("/bin/ls", "ls", "-l", NULL);     // 可执行文件的路径 /bin/ls； ls 为执行的文件  
        execl("./a.out", "a.out", NULL);     
        printf("I am child. getpid=%d, getppid=%d\n", getpid(), getppid());        
    }
    else
    {
        sleep(1);
        printf("I am parent. getpid=%d, getppid=%d\n", getpid(), getppid());        
    }
    


    return  0;
}