/*
 * @Author: JohnJeep
 * @Date: 2021-04-26 23:09:59
 * @LastEditTime: 2021-05-20 22:28:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <vector>
#include <list>
#include <deque>

using namespace std;

template<typename T, 
         template<class > 
         class Container 
        >
class Test
{
private:
   Container<T> ct;

public:
    Test() {
        for (int i = 0; i < SIZE; ++i) {
            ct.insert(ct.end(), T());
        }

        output_static_data(T());
        Container<T> ct1(ct);
        Container<T> ct2(std::move(ct));
        ct1.swap(ct2);
    }

    ~Test() {}
};

template<typename T>
using Vec = vector<T, allocator<T>>;

template<typename T>
using Lst = list<T, allocator<T>>;

template<typename T>
using Deq = deque<T, allocator<T>>;


int main(int argc, char *argv[])
{
   Test<MyString, Vec> t1; 
   Test<MyStrNoMove, Vec> t2; 
   
   Test<MyString, Lst> t1; 
   Test<MyStrNoMove, Lst> t2; 

   Test<MyString, Deq> t1; 
   Test<MyStrNoMove, Deq> t1; 
   
    return 0;
}
