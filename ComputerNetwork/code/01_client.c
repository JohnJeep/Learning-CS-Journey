/*
 * @Author: JohnJeep
 * @Date: 2020-08-20 23:10:57
 * @LastEditTime: 2020-08-22 13:02:57
 * @LastEditors: Please set LastEditors
 * @Description: 客户端程序编写
 * @FilePath: /network_programming/01_client.c
 */
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#define SERVER_PORT          9527
// #define SERVER_IP            "127.0.0.1"           
#define SERVER_IP            "192.168.1.71"          

int main(int argc, char *argv[])
{
	int cfd;	
	struct sockaddr_in server_addr; // 存储服务器的IP和port
	char buf[BUFSIZ];

	cfd = socket(AF_INET, SOCK_STREAM, 0);
	if (cfd == -1)
	{
		perror("socket error.\n");
		exit(1);
	}
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(SERVER_PORT);
	inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr.s_addr);  // 将字符串类型的点分十进制转化为网络大端字节序

	if(connect(cfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1)
	{
		perror("connect error.\n");
		exit(1);
	}

	while(1)
	{
		fgets(buf, sizeof(buf), stdin);
		write(cfd, buf, strlen(buf));             // 不能使用sizeof(buf)
		int ret = read(cfd, buf, sizeof(buf));
		write(STDOUT_FILENO, buf, ret);
	}
	close(cfd);

	return 0;
}

