#include "ConnectionPool.h"
#include <fstream>
#include <json/json.h>
#include <thread>
using namespace Json;
ConnectionPool* ConnectionPool::getConnectPool()
{
    // 对于静态局部变量的初始化，编译器自动进行lock和unlock
    static ConnectionPool pool;
    return &pool;
}

bool ConnectionPool::parseJsonFile()
{
    ifstream ifs("dbconf.json");
    Reader rd;
    Value root;
    rd.parse(ifs, root);
    if (root.isObject()) {
        m_ip = root["ip"].asString();
        m_port = root["port"].asInt();
        m_user = root["userName"].asString();
        m_passwd = root["password"].asString();
        m_dbName = root["dbName"].asString();
        m_minSize = root["minSize"].asInt();
        m_maxSize = root["maxSize"].asInt();
        m_maxIdleTime = root["maxIdleTime"].asInt();
        m_timeout = root["timeout"].asInt();
        return true;
    }
    return false;
}

/**
 * @brief 运行在独立的线程中，负责生产新的连接
 * 
 */
void ConnectionPool::produceConnection()
{
    while (true) {
        // 生产者需要访问连接队列，加锁，防止和消费者同时访问
        unique_lock<mutex> locker(m_mutexQ);
        while (m_connectionQ.size() >= m_minSize) {
            m_cond.wait(locker);   // 生产线程进入等待状态，释放锁，保证消费线程正常运行
        }

        if (m_connectionQ.size() <= m_maxSize) {
            addConnection();
        }
        m_cond.notify_all();  // 通知消费者线程去消费连接
    }
}

/**
 * @brief 回收连接。单独开一个线程不断轮询所有超时的连接，并将其释放。
 * 
 */
void ConnectionPool::recycleConnection()
{
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(500));  // 定时检查队列中超时的连接
        lock_guard<mutex> locker(m_mutexQ);   // 访问队列需要加锁
        while (m_connectionQ.size() > m_minSize) {
            MysqlConn* conn = m_connectionQ.front();
            // 判断当前的连接是否超过了最大空闲时间
            if (conn->getAliveTime() >= m_maxIdleTime) {
                m_connectionQ.pop();  // 释放连接
                delete conn;
                conn = nullptr;
            } else {
                break;
            }
        }
    }
}

/**
 * @brief 创建连接 
 * 
 */
void ConnectionPool::addConnection()
{
    MysqlConn* conn = new MysqlConn;
    conn->connect(m_user, m_passwd, m_dbName, m_ip, m_port);
    conn->refreshAliveTime();   // 记录连接创建的时间
    m_connectionQ.push(conn);
}

shared_ptr<MysqlConn> ConnectionPool::getConnection()
{
    unique_lock<mutex> locker(m_mutexQ);
    // 连接为空，阻塞等待连接超时时间
    while (m_connectionQ.empty()) {
        if (cv_status::timeout == m_cond.wait_for(locker, chrono::milliseconds(m_timeout))) { // 超时时间到了
            if (m_connectionQ.empty()) {  // 队列中仍然为空，跳过此次循环，执行下次判断
                continue;
            }
        }
    }

    // 对于使用完成的连接，不能直接销毁该连接，而是需要将该连接归还给连接池的队列，供之后的其他消费者使用
    // lambda 表达式将连接归还到连接池中
    shared_ptr<MysqlConn> connptr(m_connectionQ.front(), [this](MysqlConn* conn) {
        lock_guard<mutex> locker(m_mutexQ); // 在多线程环境中调用，要考虑队列的线程安全
        conn->refreshAliveTime();  //在归还回空闲连接到队列之前，记录下连接开始空闲的时间点
        m_connectionQ.push(conn);  // 放回队列
    });
    m_connectionQ.pop();
    m_cond.notify_all();  
    return connptr;
}

ConnectionPool::~ConnectionPool()
{
    while (!m_connectionQ.empty()) {
        MysqlConn* conn = m_connectionQ.front();
        m_connectionQ.pop();
        delete conn;
        conn = nullptr;
    }
}

ConnectionPool::ConnectionPool()
{
    // 加载配置文件
    if (!parseJsonFile()) {
        return;
    }

    for (int i = 0; i < m_minSize; ++i) {
        addConnection();
    }
    thread producer(&ConnectionPool::produceConnection, this); // 启动一个线程，负责创建连接
    producer.detach();

    thread recycler(&ConnectionPool::recycleConnection, this); // 启动一个线程，定时扫描超过最大空闲时间的连接，进行回收
    recycler.detach();
}
