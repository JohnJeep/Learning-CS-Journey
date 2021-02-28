/*
 * @Author: JohnJeep
 * @Date: 2020-08-22 16:12:54
 * @LastEditTime: 2021-02-28 13:40:23
 * @LastEditors: Please set LastEditors
 * @Description: 多线程并发编程实现
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
#include <fcntl.h>
#include <pthread.h>
#include "wrap.h"

#define MAX             8192
#define SERVER_PORT     9527

struct s_info
{
    struct sockaddr_in client_addr;
    int confd;
};


void *do_work(void *argv)
{
    struct s_info *ts = (struct s_info*)argv;
    char buf[MAX];
    char str[INET_ADDRSTRLEN];
    int i, ret;

    while (1) {
        ret = Read(ts->confd, buf, sizeof(buf));
        if (ret == 0) {
            printf("the client %d closed...\n", ts->confd);   // 关闭客户端
            break;
        }
        // printf("client IP: %s, client port: %d\n", 
        //        inet_ntop(AF_INET, &(*ts).client_addr.sin_addr, str, sizeof(str)),
        //        ntohs((*ts).client_addr.sin_port));

        for (size_t i = 0; i < ret; i++) {
            buf[i] = toupper(buf[i]);
        }
        Writen(ts->confd, buf, ret);
        Writen(STDOUT_FILENO, buf, ret);
    }
    Close(ts->confd);

    return (void *)0;
}


int main(int argc, char *argv[])
{
    int sfd, cfd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    char buf[BUFSIZ];
    char client_ip[BUFSIZ];
    pthread_t tid;
    struct s_info ts[256];    // 根据最大线程创建结构体数组
    int i = 0;

    sfd = Socket(AF_INET, SOCK_STREAM, 0);
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);    // 指定客户端任意的IP
    Bind(sfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    Listen(sfd, 128);
    while (1) {
        client_addr_len = sizeof(client_addr);
        cfd = Accept(sfd, (struct sockaddr*)&client_addr, &client_addr_len);        
        printf("client IP: %s, client port: %d\n", 
                inet_ntop(AF_INET,  &client_addr.sin_addr.s_addr, client_ip, sizeof(client_ip)),
                ntohs(client_addr.sin_port));
        ts[i].client_addr = client_addr;
        ts[i].confd = cfd; 
        pthread_create(&tid, NULL, do_work, (void*)&ts[i]);
        pthread_detach(tid);     // 子线程分离，防止僵尸线程的产生
        i++;
    }

    return 0;
}