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
 * @brief 生产连接 
 * 
 */
void ConnectionPool::produceConnection()
{
    while (true) {
        unique_lock<mutex> locker(m_mutexQ);
        while (m_connectionQ.size() >= m_minSize) {
            m_cond.wait(locker);   // 生产线程进入等待状态，释放锁，保证消费线程正常运行
        }
        addConnection();
        m_cond.notify_all();  // 通知消费者线程去消费连接
    }
}

/**
 * @brief 消费连接 
 * 
 */
void ConnectionPool::recycleConnection()
{
    while (true) {
        this_thread::sleep_for(chrono::milliseconds(500));
        lock_guard<mutex> locker(m_mutexQ);
        while (m_connectionQ.size() > m_minSize) {
            MysqlConn* conn = m_connectionQ.front();
            if (conn->getAliveTime() >= m_maxIdleTime) {
                m_connectionQ.pop();
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
    conn->refreshAliveTime();
    m_connectionQ.push(conn);
}

shared_ptr<MysqlConn> ConnectionPool::getConnection()
{
    unique_lock<mutex> locker(m_mutexQ);
    while (m_connectionQ.empty()) {
        if (cv_status::timeout == m_cond.wait_for(locker, chrono::milliseconds(m_timeout))) {
            if (m_connectionQ.empty()) {
                // return nullptr;
                continue;
            }
        }
    }
    shared_ptr<MysqlConn> connptr(m_connectionQ.front(), [this](MysqlConn* conn) {
        lock_guard<mutex> locker(m_mutexQ);
        conn->refreshAliveTime();  //在归还回空闲连接队列之前要记录一下连接开始空闲的时刻
        m_connectionQ.push(conn);
    });
    m_connectionQ.pop();
    m_cond.notify_all();  // 消费者取出一个连接之后，通知生产者，生产者检查队列，如果为空则生产
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
    thread producer(&ConnectionPool::produceConnection, this);
    thread recycler(&ConnectionPool::recycleConnection, this);
    producer.detach();
    recycler.detach();
}
