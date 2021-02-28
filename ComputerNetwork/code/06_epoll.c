/*
 * @Author: JohnJeep
 * @Date: 2020-08-23 12:21:28
 * @LastEditTime: 2021-02-28 13:32:23
 * @LastEditors: Please set LastEditors
 * @Description: 多路复用IO：epoll实现
 *               此程序为Server端的编写，采用epoll方式实现多路复用IO，client连接个数可以时任意的，
 *               epoll方式实现多路复用IO效果是最好的，select与client正常连接时，最好先连接epoll，再连接client。  
 */
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/epoll.h>

#define SERVER_PORT        9527
#define OPEN_MAX           1024

int main(int argc, char *argv[])
{
    int listenfd, sockfd, connfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len;
    char buf[BUFSIZ], client[INET_ADDRSTRLEN];
    int new_ready, epfd, res;
    struct epoll_event ep_evt, ep[OPEN_MAX];
    int num = 0;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd == -1) {
        perror("socket error");
        exit(1);
    }
    
    // 多路复用
    int opt = 1;
    if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) { 
        perror("setsockopt error");
        exit(1);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(listenfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind error");
        exit(1);
    }

    if (listen(listenfd, 128) == -1) {
        perror("listen error");
        exit(1);
    }
    
    epfd = epoll_create(OPEN_MAX);   // 创建epoll模型，ep fd指向红黑树的树根
    if (epfd == -1) {
        perror("epoll_create error");
        exit(1);
    }
     // 将lintenfd的监听事件设置为 读
    ep_evt.events = EPOLLIN;
    ep_evt.data.fd = listenfd;
    res = epoll_ctl(epfd, EPOLL_CTL_ADD, listenfd, &ep_evt); // 将listenfd和它对应的结构体设置到红黑树上
    if (res == -1) {
        perror("epoll_ctl error");
        exit(1);
    }
    
    for (; ;) {
        int i, rd_len;

        // epoll设为server阻塞监听事件，struct_epoll_event类型的数组ep, OPEN_MAX为数组容量，-1为永久阻塞
        new_ready = epoll_wait(epfd, ep, OPEN_MAX, -1);
        if (new_ready == -1) {
            perror("epoll_wait error");
            exit(1);
        }
        for (i = 0; i < new_ready; i++) {
            // 如果不是读事件，则继续判断
            if (!ep[i].events & EPOLLIN) {
                continue;
            }

            // 判断满足的事件是否时linstenfd
            if (ep[i].data.fd == listenfd) {
                client_addr_len = sizeof(client_addr);
                connfd = accept(listenfd, (struct sockaddr*)&client_addr, &client_addr_len);
                if (connfd == -1) {
                    perror("accept error");
                    exit(1);
                }
                printf("client IP: %s, IP: %d\n",
                    inet_ntop(AF_INET, (struct sockaddr*)&client_addr.sin_addr.s_addr, client, sizeof(client)),
                    ntohs(client_addr.sin_port));
                printf("connfd: %d, client: %d\n", connfd, ++num);

                ep_evt.events = EPOLLIN;
                ep_evt.data.fd = connfd;
                res = epoll_ctl(epfd, EPOLL_CTL_ADD, connfd, &ep_evt); // 将connfd和它对应的结构体设置到红黑树上
                if (res == -1) {
                    perror("epoll_ctl error");
                    exit(1);
                }
            }
            else {  // 事件不是linstenfd
                sockfd = ep[i].data.fd;
                rd_len = read(sockfd, buf, sizeof(buf));
                if (rd_len == 0) {    // 读到0，表明客户端正常关闭连接
                    res = epoll_ctl(epfd, EPOLL_CTL_DEL, sockfd, NULL); // 将文件描述符从红黑树上删除
                    if (res == -1) {
                        perror("epoll_ctl error");
                        exit(1);  
                    }
                    printf("client[%d] closed connection.\n", sockfd);

                    --num;
                    close(sockfd);  // 关闭与客户端的连接
                }
                else if (rd_len < 0) {  // read() 出错处理
                    perror("read result less 0.\n");
                    res = epoll_ctl(epfd, EPOLL_CTL_DEL, sockfd, NULL);
                    close(sockfd);
                }
                else {   // 处理正常读到的字节数
                    for (i = 0; i < rd_len; i++) {
                        buf[i] = toupper(buf[i]);       // 事件处理：小写转大写
                    }
                    write(STDOUT_FILENO, buf, rd_len);  // 写到服务器端的标准输出上
                    write(sockfd, buf, rd_len);         // 写到客户端上
                }
            }
        }
    }
    close(listenfd);
    close(sockfd);

    return 0;
}