#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
	pid_t pd;

	printf("fork before\n");
	pd = fork();
	if (pd == -1)
	{
		perror("fork create error\n");
		exit(1);
	}
	else if (pd == 0)
	{
		printf("I am child. pid=%d, ppid=%d \n", getpid(), getppid());
		sleep(3);
		printf("I am child. pid=%d, ppid=%d \n", getpid(), getppid());
	}
	else
	{
		printf("I am parent. pid=%d, ppid=%d \n", getpid(), getppid());
//		sleep(3);
	}
	printf("fork create after.\n");

	return 0;
}
