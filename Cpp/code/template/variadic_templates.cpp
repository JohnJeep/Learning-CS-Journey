/*
 * @Author: JohnJeep
 * @Date: 2021-04-23 22:27:53
 * @LastEditTime: 2021-05-05 13:13:05
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <bitset>
#include <tuple>

using namespace std;

/**
 * @description: 当包 ... 中 的个数等于0时就会执行下面这个空的函数 
 * @param {*}
 * @return {*}
 */
void my_print()
{
}

template<typename T, typename... Types>
void my_print(const T& firstAgs, const Types&... args)
{
    cout << firstAgs << endl;
    cout << "parameters of num: " << sizeof...(args) << endl; // 计算传递的参数个数
    my_print(args...);
}

// test sample2
// template<int IDX, int MAX, typename... Args>
// struct PRINT_TUPLE
// {
//     static void printX(ostream& os, const tuple<Args...>& T) {
//         os << get<IDX>(t) << (IDX + 1 == MAX ? "" : ",");
//         PRINT_TUPLE<IDX+1, MAX, Args...>::printX(os, t);
//     }
// };

// template<typename... Args>
// ostream& operator<<(ostream& os, const tuple<Args...>& t) {
//     os << "[";
//     PRINT_TUPLE<0, sizeof...(Args), Args...>::printX(os, t);
//     return os << "]";
// }

// template<int MAX, typename... Args>
// struct PRINT_TUPLE<MAX, MAX, Args...> 
// {
//     static void printX(std::ostream& os, const tuple<Args...> & t) {

//     }
// };

namespace my_tuple
{
// tuple类中递归的调用
template<typename... Values> 
  class MyTuple;

template<> class 
  MyTuple<> {};

template<typename Head, typename... Tail>
class MyTuple<Head, Tail...>
    : private MyTuple<Tail...>       // 递归继承调用
{
    typedef MyTuple<Tail...> inherited;

public:
    MyTuple() {}
    MyTuple(Head v, Tail... vtail)
        : m_head(v), inherited(vtail...) {}
    
    // typename Head::type head() {return m_head;}
    Head head() {return m_head;}
    inherited& tail() {return *this;};

protected:
    Head m_head;
};

}

int main(int argc, char *argv[])
{
    my_print(100, "hello", bitset<16>(377), 50);

    // cout << std::make_tuple(7.5, string("word"), bitset<16>(255), 100);

    my_tuple::MyTuple<int, float, string> t(41, 3.14, "nice");
   cout << sizeof(t) << endl;
   t.head();
   t.tail();
   t.tail().tail().head();

    // cout << t.head() << endl;
    // cout << t.tail() << endl;
    // cout << t.tail().tail().head() << endl;

    return 0;
}
