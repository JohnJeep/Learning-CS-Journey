/*
 * @Author: JohnJeep
 * @Date: 2020-07-14 09:21:50
 * @LastEditTime: 2021-05-23 13:19:49
 * @LastEditors: Please set LastEditors
 * @Description: 继承中异常的层次结构
 * 
 */ 
#include <iostream>
using namespace std;

class Nurse
{
private:
    int m_age;
    char *m_name;
    int m_len;
public:
    Nurse(int age);
    ~Nurse();
    char& operator[] (int index);
    int getLen();
    
    class Dimension
    {
    private:
    public:
        int m_size;

        Dimension(int d_size)
        {
            this->m_size = d_size;
        }
        virtual void showSize()
        {
            cout << m_size << endl;
        } 
        ~Dimension()
        {}
    };
    
    class Cloth:public Dimension
    {
    private:
        int c_data;
    public:
        Cloth(int c_size):Dimension(m_size = 0)
        {
            this->c_data = c_size;
        }
        virtual void showSize()
        {
            cout << "Cloth类异常：" << c_data << endl;
        } 
        ~Cloth()
        {}
    };
    
    class Stature:public Dimension
    {
    private:
        int s_data;
    public:
        Stature(int s_size):Dimension(m_size = 0)
        {
            this->s_data = s_size;
        }
        ~Stature()
        {}
        virtual void showSize()
        {
            cout << "Stature类异常：" << s_data << endl;
        } 
    };
    
    class Work:public Dimension
    {
    private:
        int w_data;
    public:
        Work(int w_size):Dimension(m_size = 0)
        {
            this->w_data = w_size;
        }
        ~Work()
        {}
        virtual void showSize()
        {
            cout << "Work类异常：" << w_data << endl;
        } 
    };
};

Nurse::Nurse(int len)
{
    this->m_len = len;
    this->m_name = new char[m_len];

    if (len < 10) {
        throw Cloth(len);   // 通过类来抛出异常
    }
    else if (len > 10 && len < 50) {
        throw Stature(len);
    }
    else if (len >50) {
        throw Work(len);
    }
}

Nurse::~Nurse()
{
    if (m_name != nullptr) {
        delete []m_name;
        m_name = nullptr;
        m_len = 0;
        cout << "执行析构函数" << endl;
    }
}

// 重载数组 [] 操作运算符
char& Nurse::operator[] (int index)
{
    return m_name[index];
}

int Nurse::getLen()
{
    return m_len;
}


int main(int argc, char *argv[])
{
    try {
        Nurse tang(20);   
        
        // 有异常跳过下面的部分
        cout << tang.getLen() << endl;
        for (int i = 0; i < tang.getLen(); i++) {
            tang[i] = i + 1;
            printf("%d ", tang[i]);
        }       
    }
    catch(Nurse::Dimension& e) {  // 异常捕获，传递的是类的引用
        e.showSize();
    }
     
    return 0;
}