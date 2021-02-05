/*
 * @Author: JohnJeep
 * @Date: 2021-01-25 20:06:45
 * @LastEditTime: 2021-02-05 16:15:33
 * @LastEditors: Please set LastEditors
 * @Description: 标准库中容器的一些测试
 */
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string>
using std::cin;
using std::cout;
using std::endl;
using std::string;

const long ASIZE  =   500000L;

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

/**
 *  array 测试
 */
#include <array>
#include <ctime>

namespace my_array
{
void test_array()
{
    cout << "test_array() ..." << endl;;

std::array<long, ASIZE> arr;     // 申请500000个long固定大小的array

clock_t timeStart = clock();
    for (long i = 0; i < ASIZE; i++) {
        arr[i] = rand();
    }

    cout << "milli-seconds : " << (clock()-timeStart) << endl;
    cout << "array.size() : " << arr.size() << endl;
    cout << "array.front() : " << arr.front() << endl;
    cout << "array.back() : " << arr.back() << endl;
    cout << "array.data() : " << arr.data() << endl;

long target = get_a_target_long();
    timeStart = clock();
    std::qsort(arr.data(), ASIZE, sizeof(long), compareLongs);

long* pItem = (long*)bsearch(&target, arr.data(), ASIZE, sizeof(long), compareLongs);
    cout <<"use qsort+bsearch(): " << (clock()-timeStart) << " milli-seconds" << endl; 

    if (pItem != nullptr) {
        cout << "find number: " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }
}

}

/**
 *  vector 测试
 */
#include <vector>
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
        std::cout << "find number: " << *pItem << std::endl;
    }
    else {
        std::cout << "not find number" << std::endl;
    }

    timeStart = clock();
    std::sort(vec.begin(), vec.end());  // STL中的全局算法
std::string* ptr = (std::string*)bsearch(&target, (vec.data()), 
                                         vec.size(), sizeof(std::string), compareStrings);

    std::cout << "sort+bsearch(), milli-seconds : " << (clock()-timeStart) << std::endl;    

    if (ptr != NULL) {
        std::cout << "find number: " << *ptr << std::endl;
    }
    else {
        std::cout << "not find number" << std::endl;
    }
}


}; // namespace my_vector

/**
 *  list 测试
 */
#include <list>
#include <ctime>
#include <string>
using std::string;

namespace my_list
{
void test_list(long& value)
{
    cout << "test_list() ..." << endl;

std::list<string> lt;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; ++i) {
        try {
            snprintf(buf, 10, "%d", rand());
            lt.push_back(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }  
    }

    cout << "milli-seconds : " << (clock()-timeStart) << endl;
    cout << "list.size() : " << lt.size() << endl;
    cout << "list.front() : " << lt.front() << endl;
    cout << "list.back() : " << lt.back() << endl;
    cout << "list.max_size() : " << lt.max_size() << endl;

string target = get_a_target_string();
    timeStart = clock();

auto pItem = find(lt.begin(), lt.end(), target);
    cout << "find(): " << clock()-timeStart << " ms" << endl;

    if (pItem != lt.end()) {
        cout << "find number: " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }
    
    timeStart = clock();
    lt.sort();
    cout << "lt.sort() : " << clock()-timeStart << " ms" << endl; 
}
}

/**
 * forward-list 测试
 */
#include <forward_list>

namespace my_forward_list
{
void test_forward_list(long& value)
{
    cout << "test_list() ..." << endl;

std::forward_list<string> lt;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; ++i) {
        try {
            snprintf(buf, 10, "%d", rand());
            lt.push_front(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }  
    }

    cout << "milli-seconds : " << (clock()-timeStart) << endl;
    cout << "list.front() : " << lt.front() << endl;
    cout << "list.max_size() : " << lt.max_size() << endl;

string target = get_a_target_string();
    timeStart = clock();

auto pItem = find(lt.begin(), lt.end(), target);
    cout << "find(): " << clock()-timeStart << " ms" << endl;

    if (pItem != lt.end()) {
        cout << "find number: " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }
    
    timeStart = clock();
    lt.sort();
    cout << "lt.sort() : " << clock()-timeStart << " ms" << endl;     
}
}

/**
 *  slist 测试
 *  slist 是gnu库中的链表
 */
#include <ext\slist>
#include <stdexcept>
#include <string>
#include <ctime>
#include <cstdlib> // abort()
#include <cstdio>
#include <iostream>

namespace my_slist
{
void test_slist(long& value)
{
    cout << "test_slist() ..." << endl;
    
__gnu_cxx::slist<string> lt;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            lt.push_front(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }
    
    cout << "Carry time: " << clock()-timeStart << " ms" << endl;
}    
}

/**
 *  deque 测试
 */
#include <deque>
#include <ctime>
#include <string>
using std::string;

namespace my_deque
{
void test_deque(long& value)
{
    cout << "test_deque() ..." << endl;

std::deque<string> deq;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; ++i) {
        try {
            snprintf(buf, 10, "%d", rand());
            deq.push_back(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }  
    }

    cout << "milli-seconds : " << (clock()-timeStart) << endl;
    cout << "deque.size() : " << deq.size() << endl;
    cout << "deque.front() : " << deq.front() << endl;
    cout << "deque.back() : " << deq.back() << endl;
    cout << "deque.max_size() : " << deq.max_size() << endl;

string target = get_a_target_string();
    timeStart = clock();

auto pItem = find(deq.begin(), deq.end(), target);
    cout << "find(): " << clock()-timeStart << " ms" << endl;

    if (pItem != deq.end()) {
        cout << "find number: " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }
    
    timeStart = clock();
    sort(deq.begin(), deq.end());   // 标准库中sort
    cout << "deq.sort() : " << clock()-timeStart << " ms" << endl; 
}
}


/**
 *  stack 测试
 */

/**
 *  queue 测试
 */

/**
 *  set 测试
 */

/**
 *  map 测试
 */


/**
 *  multiset 测试
 */

/**
 *  multimap 测试
 */

/**
 *  unordered-multiset 测试
 */

/**
 *  unordered-multimap 测试
 */

/**
 *  unordered-set 测试
 */

/**
 *  unordered-map 测试
 */

/**
 *  alloctor 测试
 */

void elapsed_time()
{
    printf("Elapsed time:%u secs.\n",clock()/CLOCKS_PER_SEC);
}

int main(int arg, char* argv[])
{
    elapsed_time();
    long value = 1000000;
    
    // my_array::test_array();

    // my_vector::test_vector(value);
    
    // my_list::test_list(value);
    
    // my_forward_list::test_forward_list(value);

    // my_slist::test_slist(value);

    my_deque::test_deque(value);
    
    return 0;
}