/*
 * @Author: JohnJeep
 * @Date: 2020-08-23 12:21:04
 * @LastEditTime: 2021-02-28 13:30:00
 * @LastEditors: Please set LastEditors
 * @Description: 多路IO复用之poll的使用
 *               此程序为Server端的编写，采用poll方式实现IO多路复用，连接个数可以超过1024个client，
 *               select与client正常连接时，最好先连接poll，再连接client。 
 */
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <arpa/inet.h>

#define SERVER_PORT           9527
#define OPEN_MAX              1024

int main(int argc, char *argv[])
{
    int listenfd, connfd, sockfd;
    int i, j, n, max_i;
    int new_read;
    struct pollfd client[OPEN_MAX];
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len;
    char buf[BUFSIZ], str[INET_ADDRSTRLEN];

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd == -1) {
        perror("socket error");
        exit(1); 
    }
    
    // 设置端口复用
    int opt = 1;
    setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));    
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if(bind(listenfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind error");
        exit(1);
    }
    
    if(listen(listenfd, 128) == -1) {
        perror("listen error");
        exit(1);
    }

    client[0].fd = listenfd;    // 将要监听的第一个文件描述符存入 lient[0] 中
    client[0].events = POLLIN;  // listenfd监听普通的我读事件

    // 将client[] 数组中剩下的元素初始化为 -1，注意：0也是文件描述符，因此不能使用
    for (i = 1; i < OPEN_MAX; i++) {
        client[i].fd = -1;
    }
    
    max_i = 0; // client[]数组中的有效元素的最大下标

    for (; ;) {
        new_read = poll(client, max_i+1, -1); // 阻塞监听是否有客户端连接请求    

        // 有读事件准备就绪
        if (client[0].revents & POLLIN) {
            client_addr_len = sizeof(client_addr); 
            connfd = accept(listenfd, (struct sockaddr*)&client_addr, &client_addr_len); // 接收客户端的连接请求，accept() 不会阻塞
            if (connfd == -1) {
                perror("accept error");
                exit(1);
            }
            printf("Reveive from IP %s port: %d\n",
                   inet_ntop(AF_INET, &server_addr.sin_addr.s_addr, str, sizeof(str)),
                   ntohs(client_addr.sin_port));

            for (i = 1; i < OPEN_MAX; i++) {
                if (client[i].fd < 0) {
                    client[i].fd = connfd; // 找到client[]中的空闲位置，就存放accept()返回的conndf 文件描述符
                    break;
                }
            }

            // 如果连接达到最大客户端数
            if (i == OPEN_MAX) {
                printf("connect too many clients.\n");
            }

            client[i].events = POLLIN; // 设置刚刚返回的connfd，监听读事件
            if (i > max_i) {
                max_i = i;   // 更新client[] 中最大元素的下标
            }
            if (--new_read <= 0) {
                continue;  // 没有更多的就绪事件，继续回到poll阻塞  
            }
        }

        for (i = 1; i < OPEN_MAX; i++) {  // 判断哪个connfd准备就绪
            if ((sockfd = client[i].fd) < 0) {
                continue;
            }
            
            if (client[i].revents & POLLIN) {
                if ((n = read(sockfd, buf, BUFSIZ)) < 0) {
                    if (errno == ECONNRESET) {
                        printf("client[%d] aborted connection.\n", i);
                        close(sockfd);
                        client[i].fd = -1;  // poll中不监控文件描述符时，直接将其设置为-1即可
                    }
                    else {
                        printf("read error");
                        exit(1);
                    }
                }
                else if (n == 0) {   // 客户端正常关闭                
                    printf("client[%d] closed connection.\n", i);
                    close(sockfd);
                    client[i].fd = -1;
                }
                else {
                    for (j = 0; j < n; j++)
                    {
                        buf[j] = toupper(buf[j]);
                    }
                    write(sockfd, buf, n);                    
                }
                
                if (--new_read <= 0) {
                    break;
                }
            }
        }
    }

    return 0;
}