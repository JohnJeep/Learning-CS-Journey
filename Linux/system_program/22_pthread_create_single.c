/*
 * @Author: JohnJeep
 * @Date: 2020-06-16 21:05:45
 * @LastEditTime: 2020-09-09 21:51:06
 * @LastEditors: Please set LastEditors
 * @Description: 创建一个线程，并得到线程号
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

void *thread_func(void *arg)
{
    pthread_t tid = pthread_self();
    printf("pthread id %lu\n", tid);
}

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret;

    printf("pthread create before, pthread id=%lu, pid=%d\n", pthread_self(), getpid());
    ret = pthread_create(&tid, NULL, thread_func, NULL);
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