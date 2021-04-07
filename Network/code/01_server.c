/*
 * @Author: JohnJeep
 * @Date: 2020-08-19 21:03:19
 * @LastEditTime: 2021-02-28 13:06:11
 * @LastEditors: Please set LastEditors
 * @Description: socket编程服务器端编写
 *               当前Server端的IP为：192.168.1.71，Port为：9527
 *               Server与client正常连接，需要先启动Server端，再启动Client端。
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

#define SERVER_IP       "192.168.1.71"  // 绑定指定的网卡IP地址  
// const char* IP = "192.168.1.71";     // inet_addr(IP)使能时调用
#define SERVER_PORT     9527            // 端口号小于65535

int main(int argc, char *agv[])
{
    int sfd, cfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_add_len;
    char buf[BUFSIZ];
    char client_ip[BUFSIZ];

    sfd = socket(AF_INET, SOCK_STREAM, 0);   // IPv4, TCP
    if (sfd == -1) {
        perror("socket error");
        exit(1);
    }

    // 新增一个端口复用，当client处于TIME_WAIT状态时，server也能重新启动，与客户端进行通信
    int opt = 1;
    if(setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        perror("setsockopt error");
        exit(1);
    }

    // INADDR_ANY: 自动获取网卡对应的有效的IP地址
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;   
    server_addr.sin_port = htons(SERVER_PORT);          // 将本地小端序的套接字转化为网络字节序
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);    // 将点分十进制以字符串类型的IP地址转化为网络字节序
   
    // server_addr.sin_addr.s_addr = inet_addr(IP);     // 指定特定的IP
   	// inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr.s_addr);

	if(bind(sfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("bind error");
        exit(1);
    }
    
    // 128为系统默认的最大值
    if(listen(sfd, 128) == -1) {
        perror("listen error"); 
        exit(1);
    }

    client_add_len = sizeof(client_addr);  // client_add_len为传入传出参数

    // accpet()函数的返回值为一个新的文件描述符cfd，客户端使用时需要用这个文件描述符
    cfd = accept(sfd, (struct sockaddr * restrict)&client_addr, &client_add_len);
    if (cfd == -1) {
        perror("accept error"); 
        exit(1);
    }

    // 打印客户端的IP和端口号
    printf("client IP: %s, client port: %d\n", 
            inet_ntop(AF_INET, &server_addr.sin_addr.s_addr, client_ip, sizeof(client_ip)),
            ntohs(client_addr.sin_port));

    while (1) {
        int ret = read(cfd, buf, BUFSIZ);
        
        if (ret == -1) {
            perror("read error");
            exit(1);
        }
        for (int i = 0; i < ret; i++) {
            buf[i] = toupper(buf[i]);   // 小写转换为大写
        }
        
        write(cfd, buf, ret);
    }
    
    close(cfd);
    close(sfd);

    return 0;
}
