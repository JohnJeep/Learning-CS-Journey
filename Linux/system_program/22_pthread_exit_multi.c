/*
 * @Author: JohnJeep
 * @Date: 2020-06-16 21:05:45
 * @LastEditTime: 2020-06-16 22:52:06
 * @LastEditors: Please set LastEditors
 * @Description: 创建多个子线程
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

void *thread_func(void *arg)
{
	// 子线程内部需要处理的部分
    int t = (int)arg;
	sleep(1);
    printf("%dth pthread id %lu, pid=%u\n", t+1, pthread_self(), getpid());
	pthread_exit(NULL);  /* 子线程退出 */
	//return NULL;
}

int main(int argc, char *argv[])
{
    pthread_t tid;
    int ret, i;

    printf("pthread create before, pthread id=%lu, pid=%d\n", pthread_self(), getpid());
    for (i = 0; i < 5; i++)
    {
        ret = pthread_create(&tid, NULL, thread_func, (void *)i);
        if (ret != 0)
        {      
            char *err = strerror(ret);     // 线程出错时，通过strerror判断
            fprintf(stderr, "pthread create failed %s.\n", err);
            exit(1);
        }        
        sleep(2);
    }
    printf("I am main pthread, pthread id=%lu, pid=%d\n", pthread_self(), getpid());
	pthread_exit(NULL);
//    return 0;  /*  进程退出 */
}
