/*
 * @Author: JohnJeep
 * @Date: 2021-08-03 21:11:38
 * @LastEditTime: 2021-08-03 22:09:01
 * @LastEditors: ubuntu
 * @Description: 线程池 C 语言实现
 * @FilePath: /cpp/threradpool.h
 */
#include "pthread.h"

// 任务结构体
typedef struct task
{
    void (*func)(void* arg); // 函数指针，需要执行的任务
    void *arg;               // 给回调函数传参数
}Task;



// 线程池结构体
struct threradpool
{
    Task* queueTask;            // 任务队列
    int queueCapacity;          // 任务队列容量
    int queueSize;              // 任务队列大小
    int queueHead;              // 任务队列队头：取数据
    int queueRear;              // 任务队列队尾：放数据
    pthread_cond_t  queueFull;  // 任务队列是否满了
    pthread_cond_t  queueEmpty; // 任务队列是否为空

    pthread_t managerId;        // 管理者线程ID
    pthread_t workerId;         // 工作线程ID
    int maxNUm;                 // 最大线程数数
    int minNum;                 // 最小线程数
    int busyNum;                // 当前工作的线程个数
    int liveNum;                // 存活的线程数
    int exitNum;                // 杀死的线程数
    bool destoryPool;           // 销毁的线程池
    pthread_mutex_t poolMutex;  // 整个线程池加锁
    pthread_mutex_t busyMutex;  // 给当前工作的线程加锁
};


