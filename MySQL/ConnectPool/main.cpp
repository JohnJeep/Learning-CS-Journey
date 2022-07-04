#include "ConnectionPool.h"
#include "MysqlConn.h"
#include <iostream>
#include <memory>
using namespace std;
// 1. 单线程: 使用/不使用连接池
// 2. 多线程: 使用/不使用连接池

void op1(int begin, int end)
{
    for (int i = begin; i < end; ++i) {
        MysqlConn conn;
        conn.connect("root", "root", "testdb", "192.168.237.131");
        char sql[1024] = { 0 };
        sprintf(sql, "insert into person values(%d, 25, 'man', 'tom')", i);
        conn.update(sql);
    }
}

void op2(ConnectionPool* pool, int begin, int end)
{
    for (int i = begin; i < end; ++i) {
        shared_ptr<MysqlConn> conn = pool->getConnection();
        char sql[1024] = { 0 };
        sprintf(sql, "insert into person values(%d, 25, 'man', 'tom')", i);
        conn->update(sql);
    }
}

void test1()
{
#if 1
    // 非连接池, 单线程, 用时: 21037278300 纳秒, 21037 毫秒
    steady_clock::time_point begin = steady_clock::now();
    op1(0, 5000);
    steady_clock::time_point end = steady_clock::now();
    auto length = end - begin;
    cout << "非连接池, 单线程, 用时: " << length.count() << " 纳秒, "
         << length.count() / 1000000 << " 毫秒" << endl;
#else
    // 连接池, 单线程, 用时: 8838406500 纳秒, 8838 毫秒
    ConnectionPool* pool = ConnectionPool::getConnectPool();
    steady_clock::time_point begin = steady_clock::now();
    op2(pool, 0, 5000);
    steady_clock::time_point end = steady_clock::now();
    auto length = end - begin;
    cout << "连接池, 单线程, 用时: " << length.count() << " 纳秒, "
         << length.count() / 1000000 << " 毫秒" << endl;

#endif
}

void test2()
{
#if 0
    // 非连接池, 多单线程, 用时: 13277417000 纳秒, 13277 毫秒
    MysqlConn conn;
    conn.connect("root", "root", "testdb", "192.168.237.131");
    steady_clock::time_point begin = steady_clock::now();
    thread t1(op1, 0, 1000);
    thread t2(op1, 1000, 2000);
    thread t3(op1, 2000, 3000);
    thread t4(op1, 3000, 4000);
    thread t5(op1, 4000, 5000);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    steady_clock::time_point end = steady_clock::now();
    auto length = end - begin;
    cout << "非连接池, 多单线程, 用时: " << length.count() << " 纳秒, "
        << length.count() / 1000000 << " 毫秒" << endl;

#else
    // 连接池, 多单线程, 用时: 3938502100 纳秒, 3938 毫秒
    ConnectionPool* pool = ConnectionPool::getConnectPool();
    steady_clock::time_point begin = steady_clock::now();
    thread t1(op2, pool, 0, 1000);
    thread t2(op2, pool, 1000, 2000);
    thread t3(op2, pool, 2000, 3000);
    thread t4(op2, pool, 3000, 4000);
    thread t5(op2, pool, 4000, 5000);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    steady_clock::time_point end = steady_clock::now();
    auto length = end - begin;
    cout << "连接池, 多单线程, 用时: " << length.count() << " 纳秒, "
         << length.count() / 1000000 << " 毫秒" << endl;

#endif
}

int query()
{
    MysqlConn conn;
    conn.connect("root", "root", "testdb", "192.168.237.131");
    string sql = "insert into person values(6, 25, 'man', 'tom')";
    bool flag = conn.update(sql);
    cout << "flag value:  " << flag << endl;

    sql = "select * from person";
    conn.query(sql);
    while (conn.next()) {
        cout << conn.value(0) << ", "
             << conn.value(1) << ", "
             << conn.value(2) << ", "
             << conn.value(3) << endl;
    }
    return 0;
}
int main()
{
    test2();
    return 0;
}