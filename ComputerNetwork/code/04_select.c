/*
 * @Author: JohnJeep
 * @Date: 2020-08-23 12:20:45
 * @LastEditTime: 2020-09-01 23:51:18
 * @LastEditors: Please set LastEditors
 * @Description: 多路复用IO：select实现
 * @FilePath: /network_programming/04_select.c
 */
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>

#define SERVER_PORT           9527

int main(int argc, char *argv[])
{
    int lfd, maxfd, sockfd, connfd;
    int max_i, n, i, j;
    int client[FD_SETSIZE];            // 遍历1024个文件描述符
    int new_read;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len;
    fd_set rset, allset;                // rset: 读事件文件描述符集合  allset: 暂存文件描述符
    char buf[BUFSIZ], str[INET_ADDRSTRLEN];

    lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd == -1)
    {
        perror("socket error.");
        exit(1);
    }
    
    memset(&server_addr, 0, sizeof(server_addr));    
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if(bind(lfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1)
    {
        perror("bind error.");
        exit(1);
    }
    if(listen(lfd, 128) == -1)
    {
        perror("listen error.");
        exit(1);
    }

    maxfd = lfd;    // 刚开始lfd为最大文件描述符
    max_i = -1;     // 将来用作client的下标，并将初值指向0个元素之前的位置
    for (i = 0; i < __FD_SETSIZE; i++)
    {
        client[i] = -1;  //  初始化client数组
    }
    FD_ZERO(&allset);   // 文件描述符清零
    FD_SET(lfd, &allset); // select所要监听的文件描述符的集合

    while (1)
    {
        rset = allset;
        new_read = select(maxfd+1, &rset, NULL, NULL, NULL); // 每次循环时，都从新设置的select监控集合开始
        if (new_read < 0)
        {
            perror("select error.");
            exit(1);
        }
        if (FD_ISSET(lfd, &rset))         // 有新的客户端发起请求
        {
            client_addr_len = sizeof(client_addr);
            connfd = accept(lfd, (struct sockaddr*)&client_addr, &client_addr_len);  // accept()不会阻塞
            if (connfd== -1)
            {
                perror("accept error.");
                exit(1);
            }
            printf("client IP: %s, client port: %d\n",
                   inet_ntop(AF_INET, &server_addr.sin_addr.s_addr, str, sizeof(str)),
                   ntohs(client_addr.sin_port));
            
            for (i = 0; i < FD_SETSIZE; i++)
            {
                if(client[i] < 0) // 找client[]中还没有使用的位置
                {
                    client[i] = connfd;  // 保存accept()返回的文件描述符到client[]中
                    break;
                }
            }
            if (i == FD_SETSIZE)  // 达到select监测的文件个数的上限
            {
                fputs("too many clients.\n", stderr);
                exit(1);
            }
            
            FD_SET(connfd, &allset); // 向监控文件描述符allset集合中添加新的文件描述符connfd
            if (connfd > maxfd)
            {
                maxfd = connfd;
            }

            if (i > max_i)
            {
                max_i = i;  // 保证max_i存的总是client[]中最后一个元素的下标
            }
            
            if (--new_read == 0)
            {
                continue;
            }
        }

        for (i = 0; i <= max_i; i++)  // 监测哪个client有数据就绪
        {
            if ((sockfd = client[i]) < 0)
            {
                continue;
            }
            if (FD_ISSET(sockfd, &rset))
            {
                n = read(sockfd, buf, sizeof(buf));
                if( n == -1)
                {
                    perror("read error.");
                    exit(1);
                }                
                if ( n == 0)  // 客户端关闭时，服务器也关闭时
                {
                    close(sockfd);
                    FD_CLR(sockfd, &allset); // 解除select对该文件描述符的监控
                    client[i] = -1;
                }
                else if (n > 0)
                {
                    for (j = 0; j < n; j++)
                    {
                        buf[j] = toupper(buf[j]);
                    }
                    sleep(1);
                    write(sockfd, buf, n);
                }                
            }
            if (--new_read == 0)
            {
                break;
            }
        }
    }
    close(lfd);

    return 0;
}