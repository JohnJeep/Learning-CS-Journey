#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <sys/wait.h>

int main(int argc, char* argv[])
{
	pid_t pd;
	int status;
	pid_t wt;
		
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
		sleep(10);
		printf("--------------child process died-------------\n");
		//exit(100);                               // 子进程正常退出的状态
		execl("./exception", "exception", NULL);   // 执行exception.c导致程序自己终止子进程
	}
	else
	{
		wt = wait(&status);
		if (wt == -1)
		{
			perror("wait error.\n");
			exit(1);
		}
		
		if (WIFEXITED(status))
		{
			printf("child exit status, status=%d\n", WEXITSTATUS(status));  // 正常退出
		}
		else if (WIFSIGNALED(status))
		{
			printf("child is killed by signal: %d\n", WTERMSIG(status));
		}

		while(1)
		{
			printf("I am parent. pid=%d, ppid=%d \n", getpid(), getppid());
			sleep(1);
		}
	}
	printf("fork create after.\n");

	return 0;
}
