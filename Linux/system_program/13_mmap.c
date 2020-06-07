/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 16:42:42
 * @LastEditTime: 2020-06-07 22:46:54
 * @LastEditors: Please set LastEditors
 * @Description: mmap()函数的实现
 * @FilePath: /system_program/13_mmap.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>

# define LEN      64

int main(int argc, char *argv[])
{
    int fd;
    char *p = NULL;
    int mun;
    int ret;
    char *str = "test mmap.\n";

    fd = open("./t_mmap.txt", O_CREAT |O_RDWR, 0664);
    if (fd == -1)
    {
        perror("open file failed.\n");
        exit(EXIT_FAILURE);
    }

    // 获取文件的大小
    ret = ftruncate(fd, LEN);
    if (ret == -1)
    {
        perror("ftruncate error.\n");
        exit(EXIT_FAILURE);
    }

    // MAP_SHARED: 内存区的数据映射到磁盘上
    p = mmap(NULL, LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);   // 映射区权限 PROT_READ|PROT_WRITE
    if (p == MAP_FAILED)
    {
        perror("MAP_FAILED error,\n");
        exit(EXIT_FAILURE);
    }
    close(fd);      // 关闭文件指针
    strcpy(p, str);
    printf("%s\n", p);

    mun = munmap(NULL, LEN);     // 释放创建的内核文件映射
    if (mun == -1)
    {   
        perror("delete mapping foe the region fail,\n");
    }

    return 0;
}


























