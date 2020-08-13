/*
 * @Author: JohnJeep
 * @Date: 2020-08-13 11:12:26
 * @LastEditTime: 2020-08-13 15:45:34
 * @LastEditors: Please set LastEditors
 * @Description: 智能指针知识
 * @FilePath: /45_smart_pointer.cpp
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


// auto_ptr pointer
void test01()
{
    auto_ptr<string> ap1(new string("I am auto pointer."));
    auto_ptr<string> ap2;
    cout << *ap1 << endl;
    ap2 = ap1;               // 此处可以执行，auto_ptr不会报错.
    // cout << *ap2 << endl; // 程序访问p1时会报错
}

// unique_ptr pointer
void test02()
{
    unique_ptr<string> up1(new string("I am unique pointer."));
    unique_ptr<string> up2;
    cout << *up1 << endl;
    // up2 = up1;      // 此时会报错！！编译不通过

    // 采用std::move()函数
    up2 = std::move(up1);
    cout << *up2 << endl;

    // 将unique指针指向的值赋给一个临时对象temp_up
    // 临时对象的生命周期不应该太长，否则编译器会禁止。
    unique_ptr<string> temp_up = unique_ptr<string>(new string("I am temp object pointer."));
    cout << *temp_up << endl;
}

// shared_ptr pointer
void test03()
{
    shared_ptr<string> sp1(new string("I am shared pointer."));
    shared_ptr<string> sp2;
    sp2 = sp1;
    cout << *sp1 << endl;
    cout << *sp2 << endl;

    // 返回内部对象指针
    cout << "sp1.get(): " << sp1.get() << endl;
    cout << "sp2.get(): " << sp2.get() << endl;

    // 返回引用计数的个数
    cout << "sp1.use_count(): " << sp1.use_count() << endl;
    cout << "sp2.use_count(): " << sp2.use_count() << endl;

    //  返回是否是独占所有权
    cout << "sp1.unique(): " << sp1.unique() << endl;
    cout << "sp2.unique(): " << sp2.unique() << endl;

    // 交换两个 shared_ptr 对象(即交换所拥有的对象)
    string *p = new string("I am string.");
    shared_ptr<string> sp3(p);
    cout << "swap before sp2: " << *sp2 << "\t" << "sp3: " << *sp3 << endl;
    swap(sp3, sp2);
    cout << "swap after sp2: " << *sp2 << "\t" << "sp3: " << *sp3 << endl;

    cout << "reset brfore sp2.use_count(): " << sp2.use_count() << endl;
    sp2.reset();        // 放弃sp2的所有权，引用计数减一
    cout << "reset after sp2.use_count(): " << sp2.use_count() << endl;

}

// weak_ptr pointer
void test04()
{
    cout << endl << "test04" << endl;
    shared_ptr<SmartPointerA> pa(new SmartPointerA());
    shared_ptr<SmartPointerB> pb(new SmartPointerB());
    cout << "pa.use_count(): " << pa.use_count() << endl;
    cout << "pb.use_count(): " << pb.use_count() << endl;

    pa->sp_b = pb;
    pb->sp_a = pa;
    cout << "pa.use_count(): " << pa.use_count() << endl;
    cout << "pb.use_count(): " << pb.use_count() << endl;

    // weak_ptr智能指针访问对象的方法，不能直接使用pa->sp_b->pFun()
    shared_ptr<SmartPointerB> fp = pa->sp_b.lock();  // 获取SmartPointerB对象的强引用
    fp->pFun();
}


int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    test04();

    return 0;
}