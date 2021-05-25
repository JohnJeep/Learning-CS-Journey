/*
 * @Author: JohnJeep
 * @Date: 2020-08-13 11:12:26
 * @LastEditTime: 2021-05-25 20:56:08
 * @LastEditors: Please set LastEditors
 * @Description: weak_ptr 例子
 */
#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

using namespace std;

class SmartPointerB;
class SmartPointerA
{
private:
    /* data */
public:
    SmartPointerA(/* args */);
    ~SmartPointerA();
    // shared_ptr<SmartPointerB> sp_b;  // 这样定义导致两个类中的指针对象相互引用，内存没有释放完
    weak_ptr<SmartPointerB> sp_b;       // 解决两个类中的指针对象相互引用，内存没有释放完的问题
};

SmartPointerA::SmartPointerA(/* args */)
{
    cout << "A excute construct function." << endl;
}

SmartPointerA::~SmartPointerA()
{
    cout << "A excute destruct function." << endl;
}

class SmartPointerB
{
private:
    /* data */
public:
    SmartPointerB(/* args */);
    ~SmartPointerB();
    shared_ptr<SmartPointerA> sp_a;
    void pFun()
    {
        cout << "I am a function" << endl;
    }
};

SmartPointerB::SmartPointerB(/* args */)
{
    cout << "B excute construct function." << endl;
}

SmartPointerB::~SmartPointerB()
{
    cout << " B excute destruct function." << endl;
}

// weak_ptr pointer
void test01()
{
    cout << "--------------------------------------" << endl;
    shared_ptr<SmartPointerA> pa(new SmartPointerA());
    shared_ptr<SmartPointerB> pb(new SmartPointerB());
    cout << "pa.use_count(): " << pa.use_count() << endl;
    cout << "pb.use_count(): " << pb.use_count() << endl;

    cout << "--------------------------------------" << endl;
    pa->sp_b = pb;
    pb->sp_a = pa;
    cout << "pa.use_count(): " << pa.use_count() << endl;
    cout << "pb.use_count(): " << pb.use_count() << endl;

    // weak_ptr智能指针访问对象的方法，不能直接使用pa->sp_b->pFun()
    cout << "--------------------------------------" << endl;
    shared_ptr<SmartPointerB> fp = pa->sp_b.lock();  // 获取SmartPointerB对象的强引用
    fp->pFun();
}

int main(int argc, char *argv[])
{
    test01();

    return 0;
}