/*
 * @Author: JohnJeep
 * @Date: 2020-06-21 15:32:38
 * @LastEditTime: 2020-06-21 16:51:52
 * @LastEditors: Please set LastEditors
 * @Description: 访问共享资源之前进行加锁，使线程能正常调度
 * @FilePath: /system_program/24_mutex.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/types.h>

pthread_mutex_t mutex;

void *tfun(void *arg)
{

    srand(time(NULL));
    while (1)
    {
        pthread_mutex_lock(&mutex);
        printf("hello ");
        sleep(rand() % 3);
        printf("word\n");
        pthread_mutex_unlock(&mutex);
        sleep(rand() % 3);
    }
}    

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret;

    srand(time(NULL));

    // 初始化锁，属性为默认属性
    pthread_mutex_init(&mutex, NULL);  
    ret = pthread_create(&tid, NULL, tfun, NULL);
    printf("ret=%d\n", ret);
    if (ret != 0)
    {   
        fprintf(stderr, "pthread create failed.\n");
        exit(1);
    }

    while (1)
    {
        pthread_mutex_lock(&mutex);
        printf("HELLO ");
        sleep(rand() % 3);
        printf("WORD\n");
        pthread_mutex_unlock(&mutex);
        sleep(rand() % 3);              // sleep() 必须在解锁之后。若在之前，子线程可能会抢不到锁，子线程会饿死，进而继续执行主控线程
    }
    
    // 销毁锁
    pthread_mutex_destroy(&mutex);

    return 0;
}

