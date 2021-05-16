/*
 * @Author: JohnJeep
 * @Date: 2021-05-16 17:44:19
 * @LastEditTime: 2021-05-16 18:44:49
 * @LastEditors: Please set LastEditors
 * @Description: pair与tuple讨论
 */
#include <iostream>
#include <cstdlib>
#include <utility>
#include <tuple>

using namespace std;

// 采用泛型模板将 pair 写入到 output stream
template<typename T1, typename T2>
std::ostream& operator<< (std::ostream& strm, const std::pair<T1, T2>& p) 
{
    return strm << "[" << p.first << "," << p.second << "]";
}

void test01_pair()
{
    typedef std::pair<int, float> intFloatPair;
    intFloatPair pl(10, 3.14);

    cout << std::get<0>(pl) << endl;   // 等价于 pl.first
    cout << std::get<1>(pl) << endl;  // yields pl.second
    cout << std::tuple_size<intFloatPair>::value << endl;  // pair 类中元素的个数
    // cout << std::tuple_element<0, intFloatPair>::type;
    // cout << tp.first << endl;
}

class Foo
{
private:
    
public:
    Foo(tuple<int, float>) 
    {
        cout << "Foo::Foo(tuple)" << endl;
    }

    template<typename... Args>
    Foo(Args... args)
    {
        cout << "Foo::Foo(args...)" << endl;
    }    

    ~Foo() {}
};

void test02_pair()
{
    tuple<int, float> t(1, 3.33);

    // 传递整个 tuple 到 Foo 的构造函数中
    pair<int, Foo> p1(42, t);

    // 传递 tuple 中的 元素到 Foo 的构造函数中
    pair<int, Foo> p2(piecewise_construct, make_tuple(55), t);

}


int main(int argc, char *argv[])
{
    test01_pair();
    test02_pair();

    return 0;
}