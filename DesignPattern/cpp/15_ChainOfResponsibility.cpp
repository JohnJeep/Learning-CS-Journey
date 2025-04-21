/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 14:39:06
 * @LastEditTime: 2020-09-16 15:11:51
 * @LastEditors: Please set LastEditors
 * @Description: 责任链模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Car
{
protected:         // 子类可以继承父类的属性
    Car* m_make;   // 指向自己的指针
public:
    Car()
    {
    }
    virtual ~Car() {}
    virtual void ProduceCarpart() = 0;
    Car* setNext(Car* make)  // 执行完一个任务后继续执行
    {
        m_make = make;
        return m_make;
    }
};

class HeadCar : public Car
{
private:
    
public:
    HeadCar() {}
    ~HeadCar() {}
    virtual void ProduceCarpart()
    {
        cout << "Produce head" << endl;
        if (m_make != nullptr)   // 造完车头后继续进行后续的铸造
        {
            m_make->ProduceCarpart();
        }
    }
};

class BodyCar : public Car
{
private:
    /* data */
public:
    BodyCar() {}
    ~BodyCar() {}
    virtual void ProduceCarpart()
    {
        cout << "produce body" << endl;
        if (m_make != nullptr)
        {
            m_make->ProduceCarpart();
        }
    }   
};

class TailCar : public Car
{
private:
    /* data */
public:
    TailCar() {}
    ~TailCar() {}
    virtual void ProduceCarpart()
    {
        cout << "produce tail" << endl; 
        if (m_make != nullptr)
        {
            m_make->ProduceCarpart();
        }
    }
};


int main(int argc, char *argv[])
{
    Car* headBenz = new HeadCar;
    Car* bodyBenz = new BodyCar;
    Car* tailBenz = new TailCar;
    
    /* 业务逻辑的实现已经写死，不利于扩展
    headBenz->ProduceCarpart();
    bodyBenz->ProduceCarpart();
    tailBenz->ProduceCarpart();
    */

    // 优化后的写法，采用责任链的方式；先确定制造的顺序 
    headBenz->setNext(tailBenz);
    tailBenz->setNext(bodyBenz);
    bodyBenz->setNext(nullptr);

    headBenz->ProduceCarpart();  // 开始制造
       
    delete headBenz;
    delete bodyBenz;
    delete tailBenz;

    return 0;
}