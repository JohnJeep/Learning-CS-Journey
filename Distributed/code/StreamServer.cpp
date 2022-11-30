#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpc/support/log.h>
#include <grpcpp/grpcpp.h>

#include "server_stream.grpc.pb.h"
#include "server_stream.pb.h"

using grpc::Server;
using grpc::ServerAsyncWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using serverstream::SimpleRequest;
using serverstream::StreamResponse;
using serverstream::StreamServer;

class ServerImpl final {
public:
    // ServerImpl();
    ~ServerImpl();

    void run();
private:
    void HandleRpcs();

private:
    std::unique_ptr<ServerCompletionQueue> m_cq;
    std::unique_ptr<Server> m_server;
    StreamServer::AsyncService m_service;

private:
    class CallData {
    public:
        CallData(StreamServer::AsyncService* service, ServerCompletionQueue* cq)
            : c_service(service)
            , c_cq(cq)
            , c_writerResponse(&c_ctx)
            , c_status(CREATE)
            , c_times(0)
        {
            Proceed();
        }

        void Proceed() 
        {
            if (c_status == CREATE) {
                c_status == PROCESS;
                // grpc 服务
                c_service->RequestListValue(&c_ctx, &c_request, &c_writerResponse, c_cq, c_cq, this); 
            } 
            else if (c_status == PROCESS) {
                if (c_times == 0) {
                    new CallData(c_service, c_cq);
                }
                if (c_times++ >= 3) {
                    c_status == FINISH;
                    c_writerResponse.Finish(Status::OK, this);
                }
                else {
                    std::string prefix("Hello");
                    c_response.set_message(prefix + c_request.data() + c_request.num_greating());
                    c_writerResponse.Write(c_response, this);   // stream 发送数据
                }
            }
            else {
                GPR_ASSERT(c_status == FINISH);
                delete this;
            }
        }
    private:
        StreamServer::AsyncService* c_service; // grpc异步运行通信的 server
        ServerCompletionQueue* c_cq;           // 异步 server 通知的生产者-消费者队列
        ServerContext c_ctx;                   // rpc context
        SimpleRequest c_request;               // 从 client 获取的请求 
        StreamResponse c_response;             // 发送给 client 的响应

        ServerAsyncWriter<StreamResponse> c_writerResponse;  // 返回 client 的方法
        int c_times;

        enum CallStatus {
            CREATE,
            PROCESS,
            FINISH
        };
        CallStatus c_status; // 当前服务的状态
    };
};

ServerImpl::~ServerImpl()
{
    m_server->Shutdown();
    m_cq->Shutdown(); // 服务关闭之后关闭完全队列
}

void ServerImpl::run()
{
    std::string server_address("127.0.0.1:8080");
    ServerBuilder builder;

    // 监听指定的IP，不需要任何的验证机制
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());

    // 注册一个 service 实例与 client 通信
    builder.RegisterService(&m_service);

    // 获取用于与 gRPC 运行时进行异步通信的完成队列
    m_cq = builder.AddCompletionQueue();

    // 组装到 server，服务开始
    m_server = builder.BuildAndStart();
    std::cout << "server listen: " << server_address << std::endl;

    // 处理 server 的主循环
    HandleRpcs();
}

void ServerImpl::HandleRpcs()
{
    new CallData(&m_service, m_cq.get());

    // 监听任务队列
    void* tag; // 请求标识
    bool ok;

    while (true) {
        GPR_ASSERT(m_cq->Next(&tag, &ok));
        GPR_ASSERT(ok);
        static_cast<CallData*>(tag)->Proceed();
    }
}


int main(int argc, char** argv)
{
    ServerImpl server;
    server.run();

    return 0;
}
