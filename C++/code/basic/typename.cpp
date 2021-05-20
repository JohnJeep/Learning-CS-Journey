/*
 * @Author: JohnJeep
 * @Date: 2020-08-28 08:42:09
 * @LastEditTime: 2020-08-28 16:09:29
 * @LastEditors: Please set LastEditors
 * @Description: typename理解
 * 
 */
#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;

template<typename T>
class Stu
{
private:
    T m_data;
public:
    Stu();
    ~Stu();
    void set(const T& var);
    void get()
    {
        cout << "m_data:" << m_data << endl;
    }

};

template<typename T>
Stu<T>::Stu()
{
}

template<typename T>
Stu<T>::~Stu()
{
}

template<typename T>
void Stu<T>::set(const T& var)
{
    this->m_data = var;
    typename T::const_iterator iter;
    // T::const_iterator iter;        // 编译出错，提示需要typename；T::const_iterator是一个嵌套从属名，
                                      // 编译时，编译器并不知道T::const_iterator是一个变量还是一种类型。
}

int main(int argc, char *argv[])
{
    Stu<string> st;
    string num = "Cat";

    st.set(num);
    st.get();

    return 0;
}
