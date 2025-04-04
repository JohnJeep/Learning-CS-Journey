/*
 * @Author: JohnJeep
 * @Date: 2020-08-11 22:12:20
 * @LastEditTime: 2020-08-11 22:22:54
 * @LastEditors: Please set LastEditors
 * @Description: 线程池例子
 *               参考：https://github.com/progschj/ThreadPool
 *                     https://github.com/mtrebi/thread-pool
 * @FilePath: /ThreadPool.cpp
 */
#include <iostream>
#include <vector>
#include <chrono>

#include "ThreadPool.h"

int main()
{
    
    ThreadPool pool(4);
    std::vector< std::future<int> > results;

    for(int i = 0; i < 8; ++i) {
        results.emplace_back(
            pool.enqueue([i] {
                std::cout << "hello " << i << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                std::cout << "world " << i << std::endl;
                return i*i;
            })
        );
    }

    for(auto && result: results)
        std::cout << result.get() << ' ';
    std::cout << std::endl;
    
    return 0;
}