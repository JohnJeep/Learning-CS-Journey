/*
 * @Author: JohnJeep
 * @Date: 2020-06-16 21:05:45
 * @LastEditTime: 2020-06-16 22:29:52
 * @LastEditors: Please set LastEditors
 * @Description: 创建一个线程，并得到线程号
 * @FilePath: /system_program/22_pthread_create.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

void *showPthreadID(void *arg)
{
    long id;
    id = pthread_self();
    printf("pthread id %lu\n", id);
}

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret;

    printf("pthread create before, pthread id=%lu, pid=%d\n", pthread_self(), getpid());

    ret = pthread_create(&tid, NULL, showPthreadID, NULL);
    if (ret != 0)
    {      
        char *err = strerror(ret);     // 线程出错时，通过strerror判断
        fprintf(stderr, "pthread create failed %s.\n", err);
        exit(1);
    }
    sleep(1);
    printf("pthread create after, pthread id=%lu, pid=%d\n", pthread_self(), getpid());


    return 0;
}


