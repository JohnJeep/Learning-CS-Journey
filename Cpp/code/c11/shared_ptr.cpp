/*
 * @Author: JohnJeep
 * @Date: 2021-05-25 20:33:12
 * @LastEditTime: 2021-05-25 22:58:07
 * @LastEditors: Please set LastEditors
 * @Description: shared_ptr 例子
 */

#include <iostream>
#include <cstdio>
#include <memory>
#include <string>

using namespace std;

// auto_ptr pointer
void test01()
{
    auto_ptr<string> ap1(new string("I am auto pointer."));
    auto_ptr<string> ap2;
    cout << *ap1 << endl;
    ap2 = ap1;               // 此处可以执行，auto_ptr不会报错.
    // cout << *ap2 << endl; // 程序访问p1时会报错
}


// shared_ptr pointer
void test02()
{
    cout << "--------------------------------------" << endl;
    shared_ptr<string> sp1(new string("I am shared pointer."));
    shared_ptr<string> sp2;
    sp2 = sp1;                // copy assignment
    cout << *sp1 << endl;
    cout << *sp2 << endl;

    // 返回内部对象指针
    cout << "--------------------------------------" << endl;
    cout << "sp1.get(): " << sp1.get() << endl;
    cout << "sp2.get(): " << sp2.get() << endl;

    // 返回引用计数的个数
    cout << "--------------------------------------" << endl;
    cout << "sp1.use_count(): " << sp1.use_count() << endl;
    cout << "sp2.use_count(): " << sp2.use_count() << endl;

    //  返回是否是独占所有权
    cout << "--------------------------------------" << endl;
    cout << "sp1.unique(): " << sp1.unique() << endl;
    cout << "sp2.unique(): " << sp2.unique() << endl;

    // 交换两个 shared_ptr 对象(即交换所拥有的对象)
    cout << "--------------------------------------" << endl;
    string *p = new string("I am string.");
    shared_ptr<string> sp3(p);
    cout << "swap before sp2: " << *sp2 << "\t" << "sp3: " << *sp3 << endl;
    swap(sp3, sp2);
    cout << "swap after sp2: " << *sp2 << "\t" << "sp3: " << *sp3 << endl;

    cout << "--------------------------------------" << endl;
    cout << "reset before sp2.use_count(): " << sp2.use_count() << endl;
    sp2.reset();        // 放弃sp2的所有权，引用计数减一
    cout << "reset after sp2.use_count(): " << sp2.use_count() << endl;

}

class Stu
{
private:
    int m_id;

public:
    Stu()
    {
        cout << "default constructor.." << endl;
    }
    Stu(int id) 
    {
        cout << "constructor, id = " << id << endl;
    }
    Stu(string name) 
    {
        cout << "costructor, name = " << name << endl;
    }
    ~Stu() 
    {
        cout << "destructor ..." << endl;
    }

    void setValue(int id)
    {
        m_id = id;
    }

    void getValue()
    {
        cout << "m_id = " << m_id << endl;
    }
};

void test03()
{
    // constructor initialize 
    shared_ptr<Stu> st1(new Stu(007));
    cout << "st1 count: " << st1.use_count() << endl;
    shared_ptr<Stu> st2(new Stu("Jack"));
    cout << "st2 count: " << st2.use_count() << endl;

    // copy constructor init
    shared_ptr<Stu> st3 = st1;
    cout << "st3 count: " << st3.use_count() << endl;

    // move constructor init
    shared_ptr<Stu> st4 = std::move(st2);
    cout << "st4 count: " << st4.use_count() << endl;

    // make_shared init
    shared_ptr<Stu> st5 = make_shared<Stu>(9527);
    cout << "st5 count: " << st5.use_count() << endl;

    // reset() 重置指针
    st3.reset();
    cout << "st3 count: " << st3.use_count() << endl;

    // reset() init
    shared_ptr<Stu> st6;
    st6.reset(new Stu("I am reset() init."));
    cout << "st6 count: " << st6.use_count() << endl;

    cout << "--------------use smart pointer------------------------" << endl;
    Stu* p = st1.get();    // 获取智能指针的原始指针
    p->setValue(100);
    p->getValue();

    st6->setValue(777);
    st6->getValue();
}
void test04()
{
    cout << "-------------Implement deleter------------------------" << endl;
    // 定义一个自己的删除器：deleter,可以选择自己不手动实现
    shared_ptr<string> str(new string("Implement my deleter"),
                          [](string* p) {
                              cout << "deleter: " << *p << endl;
                              delete p;
                          });
    str = nullptr;

    // 必须要手动实现删除器
    // shared_ptr<Stu> St7(new Stu[5]);   // 执行 5 次 构造函数，析构函数执行一次，造成内存泄漏
    shared_ptr<Stu> St7(new Stu[5], [](Stu* t){  // 改进版
        delete []t;
    });

    cout << "--------------use default_delete" << endl;
    // 可用 STL 提供的系统模板删除器函数: default_delete
    shared_ptr<Stu> st8(new Stu[5], default_delete<Stu[]>());
}

template<typename T>
shared_ptr<T> make_shared_array(size_t len)
{
    return shared_ptr<T>(new T[len], default_delete<T[]>());
}

void test05()
{
    shared_ptr<Stu> t = make_shared_array<Stu>(4);
    cout << t.use_count() << endl;
}

int main()
{
    // test01();
    // test02();
    // test03();
    // test04();
    test05();

    return 0;
}