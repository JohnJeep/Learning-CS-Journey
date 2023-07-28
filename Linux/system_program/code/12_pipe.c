/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 11:41:20
 * @LastEditTime: 2020-06-07 15:18:39
 * @LastEditors: Please set LastEditors
 * @Description: pipe管道的简单用例
 * @FilePath: /system_program/12_pipe.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <string.h>

int main(int argc, char *argv[])
{
	int pipefd[2];
	char buf[256];
	pid_t pd;
	char *str = "parent write pipe.";

	int pip = pipe(pipefd);    // 先创建管道，再创建子进程
	pd = fork();
	if (pip == -1)
	{
		perror("pipe failed.\n");
		exit(EXIT_FAILURE);
	}
	
	if (pd == -1)
	{
		perror("fork peocess failed.\n");
		exit(1);
	}
	else if (pd == 0)	// 子进程从管道中读
	{
		printf("I am child. pid=%d, ppid=%d\n", getpid(), getppid());
		close(pipefd[1]);    // 关闭pipe写端	

		int ret = read(pipefd[0], buf, sizeof(buf));   // 数组的首元素地址就是指针
		if (ret == 0)
		{
			printf("child read data from parent pipe.\n");
		}
		printf("ret=%d\n", ret);
		write(STDOUT_FILENO, buf, ret);    // STDOUT_FILENO----Standard output--1
		write(STDOUT_FILENO, "\n", 1);
		close(pipefd[0]);    // 关闭读端
	}
	else   // 父进程从管道中写
	{
		printf("I am parent. pip=%d, ppid=%d\n", getpid(), getppid());
		close(pipefd[0]);    // 关闭读端
		write(pipefd[1], str, strlen(str));
		close(pipefd[1]);    // 写完后关闭写端
		wait(NULL);          // 父进程等待子进程完成
		exit(0);
	}

	return 0;
}

