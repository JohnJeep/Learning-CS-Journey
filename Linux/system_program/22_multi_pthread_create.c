/*
 * @Author: JohnJeep
 * @Date: 2020-06-16 21:05:45
 * @LastEditTime: 2020-09-09 14:45:03
 * @LastEditors: Please set LastEditors
 * @Description: 创建多个子线程
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
    int t;

    t = *(int*)arg;
    id = pthread_self();
    printf("%d th pthread id %lu, pid=%d\n", t, id, getpid());
}

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret, i;

    // printf("pthread create before, pthread id=%lu, pid=%d\n", pthread_self(), getpid());

    for (i = 0; i < 5; i++)
    {
        ret = pthread_create(&tid, NULL, showPthreadID, (void *)i);
        if (ret != 0)
        {      
            char *err = strerror(ret);     // 线程出错时，通过strerror判断
            fprintf(stderr, "pthread create failed %s.\n", err);
            exit(1);
        }        
        sleep(2);
    }
    

    printf("pthread create after, pthread id=%lu, pid=%d\n", pthread_self(), getpid());


    return 0;
}


