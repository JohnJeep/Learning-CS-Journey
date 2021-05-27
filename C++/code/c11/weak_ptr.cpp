/*
 * @Author: JohnJeep
 * @Date: 2020-08-13 11:12:26
 * @LastEditTime: 2021-05-27 23:56:09
 * @LastEditors: Please set LastEditors
 * @Description: weak_ptr 例子
 */
#include <cstdio>
#include <iostream>
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
    weak_ptr<SmartPointerB> sp_b; // 解决两个类中的指针对象相互引用，内存没有释放完的问题
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

class Book
{
private:
    /* data */
public:
    Book(/* args */) 
    {
        cout << "Book constructor..." << endl;
    }

    ~Book() 
    {
        cout << "Book deconstructor..." << endl;
    }

    // 同一块内存会被析构两次，会产生野指针
    shared_ptr<Book> getSharedPt()
    {
        return shared_ptr<Book>(this);  
    }
};


// 改进版本，完美解决
// enable_shared_from_this() 类中使用弱引用指针对象返回一个 shared_ptr 指针的对象
class Book2 : public enable_shared_from_this<Book2>
{
private:
    /* data */
public:
    Book2(/* args */) 
    {
        cout << "Book constructor..." << endl;
    }

    ~Book2() 
    {
        cout << "Book deconstructor..." << endl;
    }

    // 同一块内存会被析构两次，会产生野指针
    shared_ptr<Book2> getSharedPt()
    {
        // 需要在类里面返回一个管理当前对象的智能指针的对象，不能直接构造一个 shared_ptr 指针的对象，
        return shared_from_this();
    }
};
/**
 * @description: weak_ptr 初始化 
 */
void test01()
{
    shared_ptr<int> st(new int()); // Create a object

    weak_ptr<int> wt1;
    cout << "wt1.use_count = " << wt1.use_count() << endl;

    weak_ptr<int> wt2(st);
    cout << "wt2.use_count = " << wt2.use_count() << endl;

    weak_ptr<int> wt3(wt1);
    cout << "wt3.use_count = " << wt3.use_count() << endl;

    weak_ptr<int> wt4 = st; // 通过隐式类型转换，shared_ptr 对象直接赋值给 weak_ptr 对象
    cout << "wt4.use_count = " << wt4.use_count() << endl;

    weak_ptr<int> t();         // 调用 weak_ptr 的默认构造，创建一个临时对象
    shared_ptr<int> sp = wt4.lock();
    cout << "sp.use_count = " << sp.use_count() << endl;
}


/**
 * @description: 测试 weak_ptr 中的 成员函数
 */
void test02()
{
    try {
        shared_ptr<string> sp(new string("hi"));                                  // create shared pointer
        weak_ptr<string> wp = sp;                                                 // create weak pointer out of it
        sp.reset();                                                               // release object of shared pointer
        cout << "wp.use_count(): " << wp.use_count() << endl;                     // prints: 0
        cout << "sp.use_count(): " << sp.use_count() << endl;                     // prints: 0
        cout << "wp.expired(): " << wp.expired() << endl;                         // prints: bool→1
        cout << "boolalpha, wp.expired(): " << boolalpha << wp.expired() << endl; // prints: true
        
        // wp 不是一个有效的对象，shared_ptr constructor 会抛出异常:  throws std::bad_weak_ptr
        shared_ptr<string> p(wp);                                                 
    }
    catch (const std::exception &e) {
        cerr << "exception: " << e.what() << endl; // prints: bad_weak_ptr
    }
}

// shared_ptr: 两个指针相互引用
void test03()
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
    shared_ptr<SmartPointerB> fp = pa->sp_b.lock(); // 获取SmartPointerB对象的强引用
    fp->pFun();
}

// 不能使用一个原始地址值初始化多个 shared_ptr。
void test04()
{
    SmartPointerA* p = new SmartPointerA;
    shared_ptr<SmartPointerA> sp1(p);
    cout << "sp1.use_count = " << sp1.use_count() << endl;
    // shared_ptr<SmartPointerA> sp2(p);    // error: 调用拷贝构造，两个智能指针指向同一个内存区，会被析构两次，产生野指针
    shared_ptr<SmartPointerA> sp3 = sp1;    // ok: 调用的是拷贝赋值操作，只执行一次析构函数
    cout << "sp2.use_count = " << sp3.use_count() << endl;   // 当前智能指针指向的内存空间中引用计数为： 2
}


/**
 * @description: 函数不能返回管理了 this 指针的 shared_ptr 对象
 * @param {*}
 * @return {*}
 */
void test05()
{
    // shared_ptr<Book> chinese(new Book);
    // cout << "chinese, use.count() = " << chinese.use_count() << endl;
    // shared_ptr<Book> poetry = chinese->getSharedPt();   
    // cout << "poetry, use.count() = " << poetry.use_count() << endl;

    shared_ptr<Book2> chinese(new Book2);
    cout << "chinese, use.count() = " << chinese.use_count() << endl;
    shared_ptr<Book2> poetry = chinese->getSharedPt();   
    cout << "poetry, use.count() = " << poetry.use_count() << endl;
}


int main(int argc, char *argv[])
{
    // test01();
    // test02();
    // test03();
    test04();
    // test05();

    return 0;
}