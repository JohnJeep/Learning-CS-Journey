/*
 * @Author: JohnJeep
 * @Date: 2020-06-15 23:09:40
 * @LastEditTime: 2020-06-15 23:36:21
 * @LastEditors: Please set LastEditors
 * @Description: 守护进程的创建
 * @FilePath: /system_program/21_daemon.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>

void create_daemon()
{
    int fd;
    pid_t pid, sid;

    // 1、创建子进程，父进程退出
    pid = fork();
    if (pid == -1)
    {
        perror("fork error.\n");
        exit(1);
    }
    if (pid > 0)
    {
        exit(1);
    }


    // 2、子进程创建会话、
    sid = setsid();
    if ((sid == -1) )
    {
        perror("setsid error.\n");
        exit(1);
    }

    // 3、改变当前目录为根目录
    fd = chdir("/home/steve");
    if (fd == -1)
    {
        perror("chdir error.\n");
        exit(1);
    }

    // 4、重设文件权限掩码
    umask(0022);

    // 5、关闭重定向文件描述符
    close(STDIN_FILENO);
    open("/dev/null", O_RDWR);
    dup2(0, STDOUT_FILENO);
    dup2(0, STDERR_FILENO);

}



int main(int argc, char *argv[])
{
    create_daemon();
    while (1)
    {
        /* code */
    }
    

    return 0;
}