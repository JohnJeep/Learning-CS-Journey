/*
 * @Author: JohnJeep
 * @Date: 2020-06-07 15:18:20
 * @LastEditTime: 2020-06-07 16:20:58
 * @LastEditors: Please set LastEditors
 * @Description: 使用管道实现父子进程间通信，完成：ls | wc –l。假定父进程实现ls，子进程实现wc
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
	pid_t pd;

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
		close(pipefd[1]);                               // 关闭pipe写端	
		int dup_t = dup2(pipefd[0], STDIN_FILENO);      // 先重定向文件描述符，再执行execlp()
		printf("dup_t=%d\n", dup_t);
		execlp("wc", "wc", "-l", NULL);                 // 执行成功后不会返回
		close(pipefd[0]);                               // 关闭读端
	}
	else   // 父进程从管道中写
	{
		printf("I am parent. pip=%d, ppid=%d\n", getpid(), getppid());
		close(pipefd[0]);    // 关闭读端
		dup2(pipefd[1], STDOUT_FILENO);
		execlp("ls", "ls", NULL);
		close(pipefd[1]);    // 写完后关闭写端
		wait(NULL);          // 父进程等待子进程完成
		exit(0);
	}

	return 0;
}

