/*
 * @Author: JohnJeep
 * @Date: 2020-08-19 21:03:19
 * @LastEditTime: 2020-08-20 00:56:21
 * @LastEditors: Please set LastEditors
 * @Description: socket编程服务器端编写
 * @FilePath: /01_server.c
 */
#include <memory.h>
#include <unistd.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>


#define SERVER_PORT     6000    // 端口号小于65535
// #define BUFFERSIZE      1024

int main(int argc, char *agv[])
{
    int sfd, cfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_add_len;
    char buf[BUFSIZ];

    sfd = socket(AF_INET, SOCK_STREAM, 0);   // IPv4, TCP
    if (sfd == -1)
    {
        perror("socket error.\n");
        exit(1);
    }

    // memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(SERVER_PORT);         // 将本地小端序的套接字转化为网络字节序
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);    // 将点分十进制以字符串类型的IP地址转化为网络字节序， INADDR_ANY: 绑定随机的IP地址
    if(bind(sfd, (struct sockaddr *)&client_addr, sizeof(server_addr)) == -1)
    {
        perror("bind error.\n");
        exit(1);
    }
    
    if(listen(sfd, 128) == -1)   // 128为系统默认的最大值
    {
        perror("bind error.\n"); 
        exit(1);
    }

    client_add_len = sizeof(client_addr);  // client_add_len为传入传出参数
    cfd = accept(sfd, (struct sockaddr * restrict)&client_addr, &client_add_len);
    if (cfd == -1)
    {
        perror("bind error.\n"); 
        exit(1);
    }
    while (1)
    {
        int ret = read(cfd, buf, BUFSIZ);
        if (ret == -1)
        {
            perror("read error.\n");
            exit(1);
        }
        for (int i = 0; i < ret; i++)
        {
            buf[i] = toupper(buf[i]);   // 小写转换为大写
        }
        
        write(cfd, buf, ret);
    }
    
    close(cfd);
    close(sfd);

    return 0;
}