/*
 * @Author: JohnJeep
 * @Date: 2020-06-21 15:32:38
 * @LastEditTime: 2020-06-21 16:45:48
 * @LastEditors: Please set LastEditors
 * @Description: 没有加锁，访问共享资源会出现调度顺序不一致，抢占资源
 * @FilePath: /system_program/24_mutex.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/types.h>

void *tfun(void *arg)
{
    srand(time(NULL));
    while (1)
    {
        printf("hello");
        sleep(rand() % 3);
        printf("word\n");
        sleep(rand() % 3);
    }
}    

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret;

    srand(time(NULL));
    ret = pthread_create(&tid, NULL, tfun, NULL);
    printf("ret=%d\n", ret);
    if (ret != 0)
    {   
        fprintf(stderr, "pthread create failed.\n");
        exit(1);
    }

    while (1)
    {
        printf("HELLO");
        sleep(rand() % 3);
        printf("WORD\n");
        sleep(rand() % 3);        
    }
    

    return 0;
}

