/*
 * @Author: your name
 * @Date: 2020-06-06 10:58:19
 * @LastEditTime: 2020-06-07 15:41:43
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /system_program/09_exec-test.c
 */ 
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc, char *argv[])
{
	pid_t pd;
	int fd;
	char *const ptr[] = {"ps", "aux", NULL}; 

	pd = fork();
	if (pd == -1)
	{
		perror("fork process failed.\n");
		exit(1);
	}
	else if (pd == 0)
	{
		printf("I am child. pid=%d, ppid=%d\n", getpid(), getppid());

		fd = open("./ps_info.txt", O_CREAT | O_RDWR | O_TRUNC, 664);
		if (fd == -1)
		{
			perror("create file failed.\n");
			exit(1);
		}
		dup2(fd, STDOUT_FILENO);
		execv("/bin/ps", ptr);
//		execlp("ps", "ps", "aux", NULL);
		fflush(stdout);
	}
	else
	{
		printf("I am parent. pid=%d, ppid=%d\n", getpid(), getppid());
	}

	return 0;
}

