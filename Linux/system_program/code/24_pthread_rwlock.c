/*
 * @Author: JohnJeep
 * @Date: 2020-06-21 17:11:19
 * @LastEditTime: 2020-06-21 18:19:34
 * @LastEditors: Please set LastEditors
 * @Description: 采用读写锁的方式对线程进行加锁
 *              线程写3个锁，读5个锁
 * @FilePath: /system_program/24_pthread_wrlock.c
 */ 
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/types.h>

int counter;
pthread_rwlock_t rwlock;

void *th_write(void *arg)
{
    int i, t;

    i = (int)arg;
    t = counter;
    while (1)
    {
        pthread_rwlock_wrlock(&rwlock);
        printf("before counter: %d\n", t);
        printf("write: %d, thread id: %lu, counter value: %d\n", i, pthread_self(), counter++);
        pthread_rwlock_unlock(&rwlock);
        sleep(2);
    }
    return NULL;
}


void *th_read(void *arg)
{
    int i;

    i = (int)arg;
    while (1)
    {
        pthread_rwlock_rdlock(&rwlock);
        printf("read: %d, thread id: %lu, counter value: %d\n", i, pthread_self(), counter);
        pthread_rwlock_unlock(&rwlock);
        sleep(2);
    }
    return NULL;   
}

int main(int argc, char *argv[])
{
    int j;
    pthread_t tid[8];

    // 初始化读写锁
    pthread_rwlock_init(&rwlock, NULL);
    for (j = 0; j < 3; j++)   // 写者锁
    {
        pthread_create(&tid[j], NULL, th_write, (void *)j);
    }

    for (j = 0; j < 5; j++)  // 读者锁
    {
        pthread_create(&tid[j], NULL, th_read, (void *)j);
    }   

    for (j = 0; j < 8; j++)  // 子进程进行回收
    {
        pthread_join(tid[j], NULL);
    }    
    pthread_rwlock_destroy(&rwlock);      // 销毁读写锁

    return 0 ;
}