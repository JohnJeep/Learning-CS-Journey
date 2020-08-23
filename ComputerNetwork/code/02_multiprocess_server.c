/*
 * @Author: JohnJeep
 * @Date: 2020-08-22 16:12:07
 * @LastEditTime: 2020-08-23 01:27:24
 * @LastEditors: Please set LastEditors
 * @Description: 多进程并发服务器端编写
 * @FilePath: /network_programming/02_multiprocess_server.c
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <ctype.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include "wrap.h"

#define SERVER_PORT        5600

void sig_handler(int signum)
{
    // 当服务在运行时，每次一个客户端从服务器断开连接时，都有一个客户端的进程（子进程）资源没有回收，会产生僵尸进程
    // 需要父进程回收子进程，来处理僵尸进程
    while (waitpid(0, NULL , WNOHANG) > 0)  // pid = 0时，回收当前进程组内所有的子进程
    {
    }
    return;    
}

int main(int argc, char *argv[])
{
    int sfd, cfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    char buf[BUFSIZ];
    char client_ip[BUFSIZ];
    pid_t pid;

    sfd = Socket(AF_INET, SOCK_STREAM, 0);
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    Bind(sfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    Listen(sfd, 128);
    while (1)
    {
        client_addr_len = sizeof(client_addr);
        cfd = Accept(sfd, (struct sockaddr*)&client_addr, &client_addr_len);        
        printf("client IP: %s, client port: %d\n", 
                inet_ntop(AF_INET,  &client_addr.sin_addr.s_addr, client_ip, sizeof(client_ip)),
                ntohs(client_addr.sin_port));

        pid = fork();
        if (pid < 0)
        {
            perr_exit("fork error.");
        }
        else if (pid == 0)
        {

            Close(sfd);   // 子进程关闭服务器端的文件描述符
            break;
        }
        else
        {
            Close(cfd);   // 父进程关闭客户端端的文件描述符
            signal(SIGCHLD, sig_handler);  // 父进程注册一个信号
        }
    }
    
    if (pid == 0)
    {
        while (1)
        {
            int ret = Read(cfd, buf, sizeof(buf));
            if (ret == 0)   // 数据读完了，客户端关闭
            {
                Close(cfd);
                return 0;
            }
            else if (ret == -1) // 出错
            {
                perror("read error.");
                exit(1);
            }
            else   // 服务器端进行数据的处理
            {
                for (size_t i = 0; i < ret; i++)
                {
                    buf[i] = toupper(buf[i]);
                }
                Write(cfd, buf, ret);
                write(STDOUT_FILENO, buf, ret);   // 写到标准输出上面
            }
        }
    }
 
    return 0;
}
