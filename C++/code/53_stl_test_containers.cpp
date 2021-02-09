/*
 * @Author: JohnJeep
 * @Date: 2021-01-25 20:06:45
 * @LastEditTime: 2021-02-09 16:07:16
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
#include <stack>
#include <string>
#include <ctime>

namespace my_stack
{
void test_stack(long& value)
{
    cout << "test_stack() ..." << endl;
    
std::stack<string> st;   
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            st.push(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "st.size() : " << st.size() << endl;    
    cout << "st.top() : " << st.top() << endl;    
    cout << "st.pop() : ";
    st.pop();    
    cout << "\nst.size() : " << st.size() << endl;    
    cout << "st.top() : " << st.top() << endl;    
}
}


/**
 *  queue 测试
 */
#include <queue>
#include <string>
#include <ctime>

namespace my_queue
{
void test_queue(long& value)
{
    cout << "test_queue() ..." << endl;
    
std::queue<string> qu;   
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            qu.push(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "qu.size() : " << qu.size() << endl;    
    cout << "qu.front() : " << qu.front() << endl;    
    cout << "qu.back() : " << qu.back() << endl;    
    cout << "qu.pop() : ";
    qu.pop();    
    cout << "\nqu.size() : " << qu.size() << endl;    
    cout << "qu.front() : " << qu.front() << endl;    
    cout << "qu.back() : " << qu.back() << endl;    
}
}


/**
 *  set 测试
 */
#include <set>
#include <ctime>
#include <string>

namespace my_set 
{
void test_set(long& value)
{
    cout << "test_set() ..." << endl;

std::set<string> st;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            st.insert(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }
    
    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "st.size() : " << st.size() << endl;   
    cout << "st.max_size() : " << st.max_size() << endl;   

string target = get_a_target_string();

    timeStart = clock();
auto pIten = find(st.begin(), st.end(), target);    
    cout << "find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIten != st.end()) {
        cout << "find : " << *pIten << endl;
    }
    else {
        cout << "not find" << endl;
    }

    // 使用set()库中封装的find() 函数
    timeStart = clock();
auto pIt = st.find(target);    
    cout << "st.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIt != st.end()) {
        cout << "set() find : " << *pIt << endl;
    }
    else {
        cout << "set() not find" << endl;
    }    
}    
}

/**
 *  multiset 测试
 */
#include <set>
#include <ctime>
#include <string>
namespace my_multiset 
{
void test_multiset(long& value)
{
    cout << "test_multiset() ..." << endl;

std::multiset<string> mst;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            mst.insert(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }
    
    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "mst.size() : " << mst.size() << endl;   
    cout << "mst.max_size() : " << mst.max_size() << endl;   

string target = get_a_target_string();

    timeStart = clock();
auto pIten = find(mst.begin(), mst.end(), target);    
    cout << "find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIten != mst.end()) {
        cout << "find : " << *pIten << endl;
    }
    else {
        cout << "not find" << endl;
    }

    // 使用mltiset()库中封装的find() 函数
    timeStart = clock();
auto pIt = mst.find(target);    
    cout << "mst.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIt != mst.end()) {
        cout << "multiset() find : " << *pIt << endl;
    }
    else {
        cout << "multiset() not find" << endl;
    }    
}    
}

/**
 *  unordered-set 测试
 */
#include <unordered_set>
#include <ctime>
#include <string>
namespace my_unordered_set
{
void test_unordered_set(long& value)
{
    cout << "test_unordered_set() ..." << endl;

std::unordered_set<string> ust;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            ust.insert(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "unordered_set.size()" << ust.size() << endl;    
    cout << "unordered_set.max_size()" << ust.max_size() << endl;    
    cout << "unordered_set.bucket_count()" << ust.bucket_count() << endl;    
    cout << "unordered_set.load_factor()" << ust.load_factor() << endl;    
    cout << "unordered_set.max_load_factor()" << ust.max_load_factor() << endl;    
    cout << "unordered_set.max_bucket_count()" << ust.max_bucket_count() << endl;    

    for (unsigned i = 0; i< 20; ++i) {
        cout << "bucket " << i << " has " << ust.bucket_size(i) << " element" << endl;
    }

string target = get_a_target_string();
    timeStart = clock();
auto pItem = find(ust.begin(), ust.end(), target);         
    cout << "::find(), milli-seconds : " << clock()-timeStart << endl;
    if (pItem != ust.end()) {
        cout << "find number : " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }

    // 使用 unordered_set中的find()
    timeStart = clock();
auto pt = ust.find(target);         
    cout << "unordered_set.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pt != ust.end()) {
        cout << "find number : " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }       
}
}

/**
 *  unordered-multiset 测试
 */
#include <unordered_set>
#include <ctime>
#include <string>
namespace my_unordered_multiset
{
void test_unordered_multiset(long& value)
{
    cout << "test_unordered_multiset() ..." << endl;

std::unordered_multiset<string> must;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; ++i) {
        try {
            snprintf(buf, 10, "%d", rand());
            must.insert(string(buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "unordered_multiset.size() : " << must.size() << endl;    
    cout << "unordered_multiset.max_size() : " << must.max_size() << endl;    
    cout << "unordered_multiset.bucket_count() : " << must.bucket_count() << endl;    
    cout << "unordered_multiset.load_factor() : " << must.load_factor() << endl;    
    cout << "unordered_multiset.max_load_factor() : " << must.max_load_factor() << endl;    
    cout << "unordered_multiset.max_bucket_count() : " << must.max_bucket_count() << endl;    

    for (unsigned i = 0; i< 20; ++i) {
        cout << "bucket #" << i << " has " << must.bucket_size(i) << " element" << endl;
    }

string target = get_a_target_string();
    timeStart = clock();
auto pItem = find(must.begin(), must.end(), target);         
    cout << "::find(), milli-seconds : " << clock()-timeStart << endl;
    if (pItem != must.end()) {
        cout << "find number : " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }

    // 使用 unordered_multiset中的find()
    timeStart = clock();
auto pt = must.find(target);         
    cout << "unordered_multiset.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pt != must.end()) {
        cout << "find number : " << *pItem << endl;
    }
    else {
        cout << "not find number" << endl;
    }    
}
}

/**
 *  map 测试
 */
#include <map>
#include <ctime>
#include <string>
namespace my_map
{
void test_map(long& value)
{
    cout << "test_map() ..." << endl;

std::map<long, string> mp;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            mp[i] = string(buf);
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }
    
    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "mp.size() : " << mp.size() << endl;   
    cout << "mp.max_size() : " << mp.max_size() << endl;   

long target = get_a_target_long();
    timeStart = clock();
auto pIt = mp.find(target);    
    cout << "mp.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIt != mp.end()) {
        cout << "map() find : " << (*pIt).second << endl;
    }
    else {
        cout << "map() not find" << endl;
    }    
}    
}


/**
 *  multimap 测试
 */
#include <map>
#include <ctime>
#include <string>
namespace my_multimap
{
void test_multimap(long& value)
{
    cout << "test_multimap() ..." << endl;

std::multimap<long, string> mp;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            // mp[i] = string(buf);   multimap() 中不能使用 [] 操作符
            mp.insert(std::pair<long, string>(i, buf));
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }
    
    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "multimap.size() : " << mp.size() << endl;   
    cout << "multimap.max_size() : " << mp.max_size() << endl;   

long target = get_a_target_long();
    timeStart = clock();
auto pIt = mp.find(target);    
    cout << "multimap.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pIt != mp.end()) {
        cout << "multimap() find : " << (*pIt).second << endl;
    }
    else {
        cout << "multimap() not find" << endl;
    }    
}    
}


/**
 *  unordered-map 测试
 *  不能使用STL中find()，只能用本数据结构中 find() 算法
 */
#include <unordered_map>
#include <ctime>
#include <string>
namespace my_unordered_map
{
void test_unordered_map(long& value)
{
    cout << "test_unordered_map() ..." << endl;

std::unordered_map<long, string> ump;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            ump[i] = string(buf);
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "unordered_map.size() : " << ump.size() << endl;    
    cout << "unordered_map.max_size() : " << ump.max_size() << endl;    

long target = get_a_target_long(); 
    timeStart = clock();
auto pt = ump.find(target);         
    cout << "unordered_map.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pt != ump.end()) {
        cout << "find number : " << (*pt).second << endl;
    }
    else {
        cout << "not find number" << endl;
    }       
}
}

/**
 *  unordered-multimap 测试
 */
#include <unordered_map>
#include <ctime>
#include <string>
namespace my_unordered_multimap
{
void test_unordered_multimap(long& value)
{
    cout << "test_unordered_multimap() ..." << endl;

std::unordered_multimap<long, string> umlp;
char buf[10];

clock_t timeStart = clock();
    for (long i = 0; i < value; i++) {
        try {
            snprintf(buf, 10, "%d", rand());
            umlp.insert(std::pair<long, string>(i, buf));
            // ump[i] = string(buf);   unordered_multimap()中不能使用 []  
        }
        catch(const std::exception& e) {
            cout << "i=" << i << " " << e.what() << endl;
            abort();
        }
    }

    cout << "milli-seconds : " << clock()-timeStart << endl;
    cout << "unordered_multimap.size() : " << umlp.size() << endl;    
    cout << "unordered_multimap.max_size() : " << umlp.max_size() << endl;    

long target = get_a_target_long();
    timeStart = clock();
auto pt = umlp.find(target);         
    cout << "unordered_multimap.find(), milli-seconds : " << clock()-timeStart << endl;
    if (pt != umlp.end()) {
        cout << "find number : " << (*pt).second << endl;
    }
    else {
        cout << "not find number" << endl;
    }       
}
}


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

    // my_deque::test_deque(value);

    // my_stack::test_stack(value);

    // my_queue::test_queue(value);

    // my_set::test_set(value);

    // my_multiset::test_multiset(value);

    // my_unordered_set::test_unordered_set(value);

    // my_unordered_multiset::test_unordered_multiset(value);

    // my_map::test_map(value);

    // my_multimap::test_multimap(value);

    // my_unordered_map::test_unordered_map(value);

    my_unordered_multimap::test_unordered_multimap(value);



    
    return 0;
}