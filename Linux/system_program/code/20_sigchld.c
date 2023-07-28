/*
 * @Author: JohnJeep
 * @Date: 2020-06-14 16:45:59
 * @LastEditTime: 2020-06-14 22:58:04
 * @LastEditors: Please set LastEditors
 * @Description: 使用sigchild进行父子间信号的通信
 * @FilePath: /system_program/20_sigchild.c
 */ 
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <stdio.h>

// 回调函数中子进程进行回收
void do_catch(int signo)
{
    pid_t wp;
    int wstatus;

    printf("catch sigchld signal signo=%d\n", signo);
    wp = waitpid(0, &wstatus, WNOHANG);
    // if (wp > 0)
    while (wp > 0)
    {
        if (wp == -1)
        {
            perror("waitpid error.\n");
            exit(1);
        }

        if (WIFEXITED(wstatus))
        {
            printf("child %d exit status %d\n", wp, WEXITSTATUS(wstatus));
        }
        else if (WIFSIGNALED(wstatus))
        {
            printf("child %d cannel siganl %d\n", wp, WTERMSIG(wstatus));
        }
    }
}

int main(int argc, char *argv[])
{
    pid_t pid;
    int i;
    int count = 10;

    for (i = 0; i < count; i++)
    {
        pid = fork();
        if (pid == -1)
        {
            perror("process fork error.\n");
            exit(1);
        }
        if (pid == 0)   // frok child process
            break;
    }

    if (pid == 0)    // 对所有的子进程做处理
    {
        int n = 1;
        while (n--)
        {
            printf("child pid %d\n", getpid());
            sleep(1);
        }
        return (i + 1);    
    }
    if (pid > 0)
    {
        struct sigaction newact;

        newact.sa_handler = do_catch;
        sigemptyset(&newact.sa_mask);
        newact.sa_flags = 0;
        sigaction(SIGCHLD, &newact, NULL);
        while (1)
        {
            printf("parent pid %d\n", getpid());
            sleep(1);
        }    
    }
    return 0;
}
