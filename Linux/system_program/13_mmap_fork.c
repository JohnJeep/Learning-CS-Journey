/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 21:57:19
 * @LastEditTime: 2020-06-07 22:43:38
 * @LastEditors: Please set LastEditors
 * @Description: mmap()实现父子间进程通信
 * @FilePath: /system_program/13_mmap.c
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
    int fd;
    int *p = NULL;
    int mun;
    int ret;
    char *str = "test mmap.\n";
    int var = 0;

    fd = open("./tmp.txt", O_CREAT | O_RDWR | O_TRUNC, 0664);
    if (fd == -1)
    {
        perror("open file failed.\n");
        exit(EXIT_FAILURE);
    }

    unlink("./tmp.txx");              // 删除临时文件目录项

    // 获取文件的大小
    ret = ftruncate(fd, LEN);
    if (ret == -1)
    {
        perror("ftruncate error.\n");
        exit(EXIT_FAILURE);
    }

    // MAP_SHARED: 内存区的数据映射到磁盘上，子进程与父进程共享内核区
    // MAP_PRIVATE: 内存区的数据映射到磁盘上，子进程与父进程各自占有一个内核区
    // p = (int *)mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);   // 映射区权限 PROT_READ|PROT_WRITE
    p = (int *)mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_PRIVATE, fd, 0);   // 映射区权限 PROT_READ|PROT_WRITE
    printf("*p=%d\n", *p);

    if (p == MAP_FAILED)
    {
        perror("MAP_FAILED error,\n");
        exit(EXIT_FAILURE);
    }
    close(fd);      // 关闭文件指针

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

        mun = munmap(NULL, LEN);     // 释放创建的内核文件映射
        if (mun == -1)
        {   
            perror("delete mapping foe the region fail.\n");
            exit(EXIT_FAILURE);
        }
    }
    



    return 0;
}


























