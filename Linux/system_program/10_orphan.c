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
		while(1)
		{
			printf("I am child. pid=%d, ppid=%d \n", getpid(), getppid());
			sleep(5);
			printf("--------------parent process died-------------\n");
		}
	}
	else
	{
		printf("I am parent. pid=%d, ppid=%d \n", getpid(), getppid());
		sleep(1);
	}
	printf("fork create after.\n");

	return 0;
}
