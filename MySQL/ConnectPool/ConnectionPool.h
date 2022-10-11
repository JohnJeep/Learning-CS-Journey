#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include "MysqlConn.h"
using namespace std;
class ConnectionPool
{
public:
    static ConnectionPool* getConnectPool();
    ConnectionPool(const ConnectionPool& obj) = delete;
    ConnectionPool& operator=(const ConnectionPool& obj) = delete;
    shared_ptr<MysqlConn> getConnection();
    ~ConnectionPool();
    
private:
    ConnectionPool();
    bool parseJsonFile();
    void produceConnection();
    void recycleConnection();
    void addConnection();

    string m_ip;
    string m_user;
    string m_passwd;
    string m_dbName;
    unsigned short m_port;
    int m_minSize;       // 连接池维护的最小连接数
    int m_maxSize;       // 连接池维护的最大连接数    没用到？
    int m_timeout;       // 连接池获取连接的超时时间
    int m_maxIdleTime;   // 连接池中连接的最大空闲时长
    queue<MysqlConn*> m_connectionQ;
    mutex m_mutexQ;      // 维护连接队列的线程安全互斥锁
    condition_variable m_cond;  // 条件变量，用于消费者和生产者之间的通信
};

