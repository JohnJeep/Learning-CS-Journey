/*
 * @Author: JohnJeep
 * @Date: 2020-06-21 11:00:53
 * @LastEditTime: 2020-06-21 11:40:48
 * @LastEditors: Please set LastEditors
 * @Description: 设置线程属性
 * @FilePath: /system_program/23_pthread_attr.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

void *show(void *arg)
{
    printf("执行子线程\n");
    return ((void *)0);   // 子线程退出
    // pthread_exit((void *)123);
}

int main(int argc, char *argv[])
{
    pthread_t tid;
    pthread_attr_t attr;
    int ret;

    // 初始化线程的属性 
    ret = pthread_attr_init(&attr);
    if (ret != 0)
    {
        fprintf(stderr, "pthread attr error %s\n", strerror(ret));
        exit(1);
    }

    // 将线程属性设置为分离状态
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    // 创建线程
    ret = pthread_create(&tid, &attr, show, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread create error %s\n", strerror(ret));
        exit(1);
    }

    // 设置子进程状态为结合态，由于已经将线程设置为分离状态，则执行pthread_join会出错
    ret = pthread_join(tid, NULL);
    if (ret != 0)
    {
        fprintf(stderr, "pthread join error: %s\n", strerror(ret));
        exit(1);
    }
    printf("pthread_join ret=%d\n", ret);
    sleep(1);

    return 0;
}


