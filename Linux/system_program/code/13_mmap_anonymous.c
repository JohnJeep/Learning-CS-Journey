/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 21:57:19
 * @LastEditTime: 2020-06-08 21:29:32
 * @LastEditors: Please set LastEditors
 * @Description: mmap()实现父子间进程通信，采用匿名映射而非文件的方式
 *               MAP_SHARED: 内存区的数据映射到磁盘上，子进程与父进程共享内核区
 *               MAP_PRIVATE: 内存区的数据映射到磁盘上，子进程与父进程各自占有一个内核区
 *               MAP_ANONYMOUS: 方式只能在Linux系统中使用，不能再类Unix中使用
 *               使用系统自带的字符文件: /dev/zero 或/dev/null，实现匿名映射。
 * 
 * @FilePath: /system_program/13_mmap_anonymous.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>

# define LEN      64

int main(int argc, char *argv[])
{
    int *p = NULL;
    int mun;
    char *str = "test mmap.\n";
    int var = 0;
    int fd;

    fd = open("/dev/zero", O_RDWR);
    if (fd == -1)
    {
        perror("open /dev/zero file failed.\n");
        exit(EXIT_FAILURE);
    }

    // p = (int *)mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);   
    // p = (int *)mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0);   
    p = (int *)mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);   
    printf("*p=%d\n", *p);

    if (p == MAP_FAILED)
    {
        perror("MAP_FAILED error,\n");
        exit(EXIT_FAILURE);
    }

    pid_t pid = fork();     // 创建子进程
    if (pid == -1)
    {
        perror("fork process failed.\n");
        exit(EXIT_FAILURE);
    }
    else if (pid == 0)
    {
        printf("I am child.");

        *p = 200;
        var = 100;
        printf("*p=%d, var=%d \n", *p, var);
    }
    else
    {
        sleep(1);
        printf("I am parent.");
        printf("*p=%d, var=%d \n", *p, var);
        wait(NULL);
        close(fd);

        mun = munmap(p, LEN);     // 释放创建的内核文件映射
        if (mun == -1)
        {   
            perror("delete mapping foe the region fail.\n");
            exit(EXIT_FAILURE);
        }
    }
    
    return 0;
}