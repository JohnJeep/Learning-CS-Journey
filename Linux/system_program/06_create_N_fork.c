/*
 * @Author: JohnJeep
 * @Date: 2020-06-02 22:02:36
 * @LastEditTime: 2020-06-04 22:25:13
 * @LastEditors: Please set LastEditors
 * @Description: 创建N个子进程例子
 */ 
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>

#define  N           5 

int main(int argc, char *argv[])
{
	pid_t pd;
	int i = 0;

	printf("fork before\n");
	for (i = 0; i < N; i++)
	{
		pd = fork();
		if (pd == -1)
		{
			perror("fork create error\n");
			exit(1);
		}
		if (pd == 0)
		{
			break;
		}
		// if (pd > 0)
		// {
		// 	printf("I am %d parent. pid=%d, ppid=%d, pd=%d \n", i+1, getpid(), getppid(), pd);
		// }	
	}
	if (i < 5)
	{
		sleep(i);
		printf("I am %d child. pid=%d, ppid=%d \n", i+1, getpid(), getppid());
	}
	else
	{
		sleep(i);
		printf("I am %d parent. pid=%d, ppid=%d \n", i, getpid(), getppid());
	}
	
	// printf("fork create after.\n");

	return 0;
}
