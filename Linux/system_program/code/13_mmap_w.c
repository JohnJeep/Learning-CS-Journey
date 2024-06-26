/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 22:45:49
 * @LastEditTime: 2020-06-08 20:57:44
 * @LastEditors: Please set LastEditors
 * @Description: 两个文件之间实现共享：mmap_w.c和mmap_r.c，
 *               即两个无血缘关系进程之间的通信。
 * @FilePath: /system_program/13_mmap_w.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/types.h>

typedef struct
{
    char name[10];
    int id;
    int sex;
}STU;


int main(int argc, char *argv[])
{
    STU student = {"lucy", 18, 1};
    int fd = 0;
    char *p = NULL;
    int len = sizeof(student);
    
    fd = open("./m_w.txt", O_CREAT | O_RDWR, 0644);
    if (fd == -1)
    {
        perror("open file failed.\n");
        exit(1);
    }
    ftruncate(fd, len);      // 参数fd指定的文件大小改为参数len指定的大小

    p = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED)
    {
        printf("mmap failed,\n");
        exit(EXIT_FAILURE);
    }
    close(fd);                      // 关闭文件描述符

    while(1)
    {
        memcpy(p, &student, len);
        student.id++;
        printf("name: %s, id: %d, sex: %d\n", student.name, student.id, student.sex);
        sleep(1);
    }

    int mup = munmap(p, len);    // 释放内核映射区
    if (mup == -1)
    {
        perror("delete munmap failed.\n");
        exit(EXIT_FAILURE);
    }

    return 0;
}
