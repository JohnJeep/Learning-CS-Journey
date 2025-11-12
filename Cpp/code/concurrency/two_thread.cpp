#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

// 开两个线程，一个线程负责按照顺序发送1-10，另一个线程负责按照顺序接收，并在终端上打印
// build: g++ -std=c++11 -pthread -o thread_comm two_thread.cpp

class MyThread {
public:
    // 发送线程
    void sender() {
        for (int i = 1; i <= max_number; ++i) {
            {
                std::lock_guard<std::mutex> lock(m_mtx);
                m_data_queue.push(i);
                // std::cout << "发送: " << i << std::endl;
            }
            m_cv.notify_one();  // 通知接收线程
        }
        
        // 发送完成信号
        {
            std::lock_guard<std::mutex> lock(m_mtx);
            m_finished = true;
        }
        m_cv.notify_one(); // 通知接收线程发送已完成
    }

    // 接收线程
    void receiver() {
        while (true) {
            std::unique_lock<std::mutex> lock(m_mtx);
            
            // 等待有数据可接收或发送完成
            m_cv.wait(lock, [this]() { 
                return !m_data_queue.empty() || m_finished; 
            });
            
            // 处理所有可用的数据
            while (!m_data_queue.empty()) {
                int value = m_data_queue.front();
                m_data_queue.pop();
                std::cout << "接收: " << value << std::endl;
            }
            
            // 如果发送完成且没有剩余数据，则退出
            if (m_finished && m_data_queue.empty()) {
                break;
            }
        }
    }

    // 运行函数
    void run() {
        std::thread send_thread(&MyThread::sender, this);
        std::thread recv_thread(&MyThread::receiver, this);
        
        send_thread.join();
        recv_thread.join();
    }
private:
    std::mutex m_mtx;
    std::condition_variable m_cv;
    std::queue<int> m_data_queue;
    bool m_finished = false;
    const int max_number = 10;
};

int main() {
    MyThread tc;
    tc.run();
    
    std::cout << "所有数字已发送和接收完成！" << std::endl;
    return 0;
}
