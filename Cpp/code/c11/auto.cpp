/*
 * @Author: JohnJeep
 * @Date: 2021-05-24 22:32:01
 * @LastEditTime: 2021-05-24 23:19:25
 * @LastEditors: Please set LastEditors
 * @Description: auto 关键字
 */
#include <iostream>
#include <cstdlib>

using namespace std;

class T1
{
private:
    /* data */
public:
    T1(/* args */) {}
    ~T1() {}

    static int get()
    {
        int num = 100; 
        cout << "I am T1" << endl;
        return num;
    }
};

class T2
{
private:
    /* data */
public:
    T2(/* args */) {}
    ~T2() {}

    // 想要在类的外部直接通过类的方法去调用的成员函数，而不是通过类的对象去调用成员函数，
    // 因此函数设计为静态成员函数
    static string get()   
    {
        const string str = "I am T2";
        return str;
    }
};

template<typename T>
void func()
{
    // 类模板中使用 auto 自动推导变量的返回类型
    auto ret = T::get();   // 不是通过类的对象去调用
    cout << ret << endl;
}


int main(int argc, char *argv[])
{
    func<T1>();
    func<T2>();
    
    return 0;
}
