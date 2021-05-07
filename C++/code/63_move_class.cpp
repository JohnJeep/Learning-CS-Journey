/*
 * @Author: JohnJeep
 * @Date: 2021-05-06 19:36:34
 * @LastEditTime: 2021-05-07 19:10:52
 * @LastEditors: Please set LastEditors
 * @Description: test move class 
 */
#include <iostream>
#include <cstdlib>
#include <vector>
#include <list>
#include <deque>
#include <map>
#include <cstring>
#include <ctime>

using namespace std;

class MyString
{
public:
    static size_t default_ctor;
    static size_t ctor;
    static size_t copy_ctor;
    static size_t copy_assign_ctor;
    static size_t move_ctor;
    static size_t move_assign_ctor;
    static size_t dector;

private:
    char* m_data;
    size_t m_len;

    void init_data(const char* s)
    {
        m_data = new char[m_len + 1];
        memcpy(m_data, s, m_len);
        m_data[m_len] = '\0';
    }
public:
    // default constructor
    MyString()
      : m_data(NULL), m_len(0) 
    {
        ++default_ctor;
    }

    // constructor
    MyString(const char* p)
      : m_len(strlen(p))
    {
        ++ctor;
        init_data(p);
    }

    // copy constructor
    MyString(const MyString& str)
      : m_len(str.m_len)
    {
        ++copy_ctor;
        init_data(str.m_data);
    }
    
    // move constructor
    MyString(MyString&& str) noexcept
      : m_data(str.m_data), m_len(str.m_len)
    {
        ++move_ctor;
        str.m_len = 0;
        str.m_data = NULL;
    }

    //copy assignment
    MyString& operator=(const MyString& str)
    {
        ++copy_assign_ctor;

        if (this != &str) {
            if (m_data) {
                delete m_data;
            }
            m_len = str.m_len;
            init_data(str.m_data);
        }
        return *this;
    }

    // move assignment, careful: MyString&& not use const keyword
    MyString& operator=(MyString&& str) noexcept
    {
        ++move_assign_ctor;

        if (this != &str) {
            if (m_data) {
                delete m_data;
            }

            // move
            m_len = str.m_len;
            m_data = str.m_data;
            str.m_len = 0;
            str.m_data = NULL;  // very important
        }
        return *this;
    }

    virtual ~MyString() 
    {
        ++dector;
        if (m_data) {
            delete m_data;
        }
    }

    bool operator< (const MyString& rhs) const
    {
        // use STL string class, compare the size
        return string(this->m_data) < string(rhs.m_data);
    }

    bool operator== (const MyString& rhs) const
    {
        return string(this->m_data) == string(rhs.m_data);
    }

    char* get() const 
    {
        return m_data;
    }
};

size_t MyString::default_ctor = 0;
size_t MyString::ctor = 0;
size_t MyString::copy_ctor = 0;
size_t MyString::copy_assign_ctor = 0;
size_t MyString::move_ctor = 0;
size_t MyString::move_assign_ctor = 0;
size_t MyString::dector = 0;

template<typename T>
void out_static_data(const T& str)
{
    cout << typeid(str).name() << "---" << endl;
    cout << "default_ctor=" << T::default_ctor << "\t"
         << "ctor=" << T::ctor << "\t"
         << "copy_ctor=" << T::copy_ctor << "\t"
         << "copy_assign_ctor=" << T::copy_assign_ctor << "\t"
         << "move_ctor=" << T::move_ctor << "\t"
         << "move_assign_ctor=" << T::move_assign_ctor << "\t"
         << "dector=" << T::dector << "\t"
         << endl; 
}

// template<typename M, typename N>
// void test_moveable(M c1, N c2, long& value)
template<typename M>
void test_moveable(M c1,  long& value)
{
    char buf[10];

    typedef typename iterator_traits<typename M::iterator>::value_type v_type;
    clock_t time_start = clock();

    for (long i = 0; i < value; ++i) {
        snprintf(buf, 10, "%d", rand());
        auto it = c1.end(); // 定位尾端
        c1.insert(it, v_type(buf));
    }
    cout << "Spend milli-seconds: " << (clock() - time_start) << endl;
    cout << "size()= " << c1.size() << endl;
    out_static_data(*(c1.begin()));

    M c11(c1);
    M c12(std::move(c1));
    c11.swap(c12);
}

int main(int argc, char *argv[])
{
    long data = 3000000;
   test_moveable(vector<MyString>(), data); 
    return 0;
}