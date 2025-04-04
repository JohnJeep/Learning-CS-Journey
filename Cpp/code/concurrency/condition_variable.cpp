/**
 * @file condition_variable.cpp
 * @author your name (you@domain.com)
 * @brief  多线中条件变量使用
 * @version 0.1
 * @date 2023-03-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

bool readyFlag;
std::mutex readyMutex;
std::condition_variable readyCondVar;

void thread_1()
{
    std::cout << "thrad_1 lock before, readFlag=" << readyFlag << std::endl;
    {
        std::lock_guard<std::mutex> lg(readyMutex);
        readyFlag = true;
    } // release lock
    readyCondVar.notify_one(); // 注：通知动作不需要放在 lock 的保护区内
}

void thread_2()
{
    {
        std::unique_lock<std::mutex> ul(readyMutex);

        // 这里必须用 unique_lock 锁而不能用 lock_guard 锁，因为 wait() 函数内部实现中明确指出用的 unique_lock
        readyCondVar.wait(ul, [] { return readyFlag; }); // 一边检查条件，一边等待通知
        std::cout << "thread_2 release lock before, readyFlag=" << readyFlag << std::endl;
    } // release lock

    std::cout << "thread_2 release lock after, readyFlag=" << readyFlag << std::endl;
    std::cout << "thread_2 done \n" << std::endl;
}

int main(int argc, char* argv[])
{
    for (int i = 0; i < 50; i++) {
        readyFlag = false;
        auto th1 = std::async(std::launch::async, thread_1);
        auto th2 = std::async(std::launch::async, thread_2);
    }

    return 0;
}