/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 15:13:05
 * @LastEditTime: 2020-09-16 15:47:17
 * @LastEditors: Please set LastEditors
 * @Description: 策略模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Strategy
{
private:
    /* data */
public:
    Strategy(/* args */) {}
    virtual ~Strategy() {}
    virtual void encrypt() = 0;
};

class AES : public Strategy
{
private:
    /* data */
public:
    AES(/* args */) {}
    ~AES() {}
    virtual void encrypt()
    {
        cout << "Asymmetric encrypt code" << endl;
    }
};

class DES : public Strategy
{
private:
    /* data */
public:
    DES(/* args */) {}
    ~DES() {}
    virtual void encrypt()
    {
        cout << "Symmetric encrypt code" << endl;
    }
};

class EncryptContent
{
private:
    Strategy* m_st;
public:
    EncryptContent(/* args */) {}
    ~EncryptContent() {}
    void setStrategy(Strategy* st)
    {
        this->m_st = st;
    }
    void work()
    {
        m_st->encrypt();
    }
};

int main(int argc, char *argv[])
{
    Strategy* sy = new AES;
    EncryptContent* enc = new EncryptContent;
    enc->setStrategy(sy);  // 先设置策略，再执行要处理的内容
    enc->work();
    delete enc;
    delete sy;

    Strategy* des = new DES;
    EncryptContent* denc = new EncryptContent;
    denc->setStrategy(des);  // 先设置策略，再执行要处理的内容
    denc->work();
    delete denc;
    delete des;

    return 0;
}