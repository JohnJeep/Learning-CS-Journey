/*
 * @Author: JohnJeep
 * @Date: 2021-01-25 20:06:45
 * @LastEditTime: 2021-01-25 22:47:54
 * @LastEditors: Please set LastEditors
 * @Description: 标准库中容器的一些测试
 */
#include <iostream>
#include <cstdlib>
using std::cin;
using std::cout;
using std::string;

long get_a_target_long()
{
long target = 0;
    cout << "\ntarget (0-" << RAND_MAX << "): ";
    cin >> target;

    return target;
}

string get_a_target_string()
{
long target = 0;
char buf[10];

    cout << "\ntarget (0-" << RAND_MAX << "): ";
    cin >> target;
    snprintf(buf, 10, "%d", target);
    
    return string(buf);
}

int compareLongs(const void* a, const void* b)
{
    return (*(long*)a - *(long*)b);
}

int compareStrings(const void* str1, const void* str2)
{
    if (*(string*)str1 > *(string*)str2) {
        return 1;
    }
    else if (*(string*)str1 < *(string*)str2) {
        return -1;
    }
    else {
        return 0;
    }
}

#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <ctime>
#include <algorithm>
#include <stdexcept>
namespace my_vector
{
void test_vector(long& value) 
{
    std::cout << "test_vector().......\n"; 

std::vector<std::string> vec;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            vec.push_back(std::string(buf));
        }
        catch(const std::exception& e) {
            std::cout << "i=" << i << " " << e.what() << std::endl;  // 防止内存被占完
            abort();
        }
    }
    std::cout << "milli-seconds : " << (clock()-timeStart) << std::endl;
    std::cout << "vector.size() : " << vec.size() << std::endl;
    std::cout << "vector.front() : " << vec.front() << std::endl;
    std::cout << "vector.back() : " << vec.back() << std::endl;
    std::cout << "vector.data() : " << vec.data() << std::endl;
    std::cout << "vector.capacity() : " << vec.capacity() << std::endl;

std::string target = get_a_target_string();
    timeStart = clock();

auto pItem = ::std::find(vec.begin(), vec.end(), target);
    std::cout << "::find(), milli-seconds : " << (clock()-timeStart) << std::endl;  // STl中的全局算法

    if (pItem != vec.end()) {
        std::cout << "find: " << *pItem << std::endl;
    }
    else {
        std::cout << "not find" << std::endl;
    }

    timeStart = clock();
    std::sort(vec.begin(), vec.end());  // STl中的全局算法
std::string* ptr = (std::string*)bsearch(&target, (vec.data()), 
                                         vec.size(), sizeof(std::string), compareStrings);

    std::cout << "sort+bsearch(), milli-seconds : " << (clock()-timeStart) << std::endl;    

    if (ptr != NULL) {
        std::cout << "find: " << *ptr << std::endl;
    }
    else {
        std::cout << "not find" << std::endl;
    }
}


}; // namespace my_vector

int main(int arg, char* argv[])
{
    long value = 1000000;
    my_vector::test_vector(value);

    return 0;
}