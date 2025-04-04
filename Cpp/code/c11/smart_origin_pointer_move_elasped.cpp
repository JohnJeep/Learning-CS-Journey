/**
 * @file smart_origin_pointer_move_elasped.cpp
 * @author JohnJeep
 * @brief  测试复杂对象分别采用智能指针、裸指针和是否使用std::move() 方法后，
 *         程序运行的时间
 * @version 0.1
 * @date 2023-05-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <ctime>
#include <chrono>
#include <sys/resource.h>
#include <malloc.h>

using namespace std;

typedef struct ThingPropertyTag {
    std::string deviceId;
    std::string attrName;
    int num;
} ThingProperty;


void PrintMallocMemoryUsage()
{
    // 调用 mallinfo() 函数获取堆内存的使用情况
    struct mallinfo info = mallinfo();

    std::cout << "总分配空间：" << info.arena << " bytes" << std::endl;
    std::cout << "空闲空间：" << info.fordblks << " bytes" << std::endl;
    std::cout << "空闲块数量：" << info.ordblks << std::endl;
    std::cout << "Total used by in-use allocated space (bytes): " << info.uordblks << std::endl;
    // std::cout << "Number of mmapped regions: " << info.hblks << std::endl;   // using mmap
    // std::cout << "Space allocated in mmapped regions (bytes): " << info.hblkhd << std::endl;// using mmap
    // std::cout << "Maximum total allocated space (bytes): " << info.usmblks << std::endl; //  only in nonthreading environments

    std::cout << std::endl;
}


/**
 * @brief 获取当前进程的内存使用情况，并打印相关信息，操作系统级别的
 * 
 */
void PrintMemoryUsage()
{
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) == 0)
    {
        std::cout << "内存使用情况：" << std::endl;
        // std::cout << "shared memory size: " << usage.ru_ixrss << " KB" << std::endl;
        // std::cout << "unshared data size: " << usage.ru_idrss << " KB" << std::endl;
        // std::cout << "unshared stack size: " << usage.ru_isrss << " KB" << std::endl;
        std::cout << "最大常驻内存集：" << usage.ru_maxrss << " KB" << std::endl;
        std::cout << "block input operations: " << usage.ru_inblock << " KB" << std::endl;
        std::cout << "block output operations: " << usage.ru_oublock << " KB" << std::endl;
        std::cout << "hard page faults: " << usage.ru_majflt << std::endl;
        std::cout << "soft page faults: " << usage.ru_minflt << std::endl;
        std::cout << "voluntary context switches: " << usage.ru_nvcsw << std::endl;
        std::cout << "involuntary context switches: " << usage.ru_nivcsw << std::endl;
    }
    else
    {
        std::cerr << "无法获取内存使用情况" << std::endl;
    }
    std::cout << std::endl;
}

/**
 * @brief 测试系统提供的高精度时钟的精度
 * 
 */
void test01()
{
    typedef std::chrono::high_resolution_clock Clock;

    cout << "num: " << Clock::period::num << "\t" << Clock::period::den << endl;
    cout << "precision: " << static_cast<double>(Clock::period::num) / Clock::period::den << endl; // 单位秒
}

void test02()
{
    // time_t start = time(nullptr);
    // std::clock_t start = std::clock();
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<ThingProperty> thing;
    thing = std::make_shared<ThingProperty>();
    thing->attrName = "comp";
    thing->deviceId = "google";
    thing->num = 100;

    std::vector<std::shared_ptr<ThingProperty>> vec;
    vec.emplace_back(thing);

    for (auto it : vec) {
        cout << it->attrName << "\t"
             << it->deviceId << "\t"
             << it->num << endl;
    }
    // time_t end = time(nullptr);
    // auto elapsedTime = std::difftime(end, start);

    // std::clock_t end = std::clock();
    // auto elapsedTime = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    // cout << "elapsed: " << elapsedTime << endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = end - start;
    cout << "has pointer no move, elapsed: " << elapsedTime.count() << endl;
}

void test03()
{
    auto start = std::chrono::high_resolution_clock::now();

    std::shared_ptr<ThingProperty> thing;
    thing = std::make_shared<ThingProperty>();
    thing->attrName = "comp";
    thing->deviceId = "google";
    thing->num = 100;

    std::vector<std::shared_ptr<ThingProperty>> vec;
    vec.emplace_back(std::move(thing));

    for (auto it : vec) {
        cout << it->attrName << "\t"
             << it->deviceId << "\t"
             << it->num << endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = end - start;
    cout << "has pointer with move, elapsed: " << elapsedTime.count() << endl;
}

void test04()
{
    auto start = std::chrono::high_resolution_clock::now();

    ThingProperty thing;
    thing.attrName = "comp";
    thing.deviceId = "google";
    thing.num = 100;

    std::vector<ThingProperty> vec;
    vec.emplace_back(thing);

    for (auto it : vec) {
        cout << it.attrName << "\t"
             << it.deviceId << "\t"
             << it.num << endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = end - start;
    cout << "no pointer no move, elapsed: " << elapsedTime.count() << endl;
}

void test05()
{
    auto start = std::chrono::high_resolution_clock::now();

    ThingProperty thing;
    thing.attrName = "comp";
    thing.deviceId = "google";
    thing.num = 100;

    std::vector<ThingProperty> vec;
    vec.emplace_back(std::move(thing));

    for (auto it : vec) {
        cout << it.attrName << "\t"
             << it.deviceId << "\t"
             << it.num << endl;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsedTime = end - start;
    cout << "no pointer with move, elapsed: " << elapsedTime.count() << endl;
}


int main()
{
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();

    test01();
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();

    test02();
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();
    
    test03();
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();

    test04();
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();

    test05();
    // PrintMemoryUsage();
    PrintMallocMemoryUsage();

    return 0;
}