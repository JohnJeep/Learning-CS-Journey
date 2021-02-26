/*
 * @Author: JohnJeep
 * @Date: 2021-02-26 09:57:47
 * @LastEditTime: 2021-02-26 16:07:39
 * @LastEditors: Please set LastEditors
 * @Description: 测试i++与++i的效率问题
 */
#include <iostream>
#include <cstdio>
#include <ctime>
#include <unistd.h>

using namespace std;

class Stu
{
private:
    long long m_id;
public:
    Stu(/* args */);
    ~Stu();

    long long get() const {return m_id;}
    void set(long long id) { m_id = id;}
};

Stu::Stu(/* args */)
{
}

Stu::~Stu()
{
}


int main(int argc, char *argv[])
{
    clock_t startTime = clock();
    Stu wang;

    for (long long i = 0; i < 1000000000; i++)
    // for (long long i = 0; i < 1000000000; ++i)
    {
        long long temp = i * i;
        wang.set(temp);     
    }
    cout << "id = " << wang.get() << endl;
    cout << "elapse time: " << clock()-startTime << endl;
    
    return 0;
}



