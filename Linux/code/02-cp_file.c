#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define BUFF_SIZE         4096

int main(int argc, char* argv[])
{
	if(argc != 3)
	{
		perror("param error\n");
		return 1;
	}

	int src_fd = open(argv[1], O_RDONLY);
	if(src_fd == -1)
	{
		perror("open");
		return 1;
	}

	int dest_fd = open(argv[2], O_CREAT | O_WRONLY, 0666);
	if(dest_fd == -1)
	{
		perror("open");
		return 1;
	}

	int len = 0;
	char buffer[BUFF_SIZE] = {0};  // 每次从文件中读取4K bytes

	// 每次文件读取成功，返回值为BUFF_SIZE
	// 读取到文件末尾时，返回一个 0 值
	while((len = read(src_fd, buffer, BUFF_SIZE)) > 0)
	{
		if(write(dest_fd, buffer, len)  != len)   // 向文件中写入数据
		{
			perror("write error!\n");
			return 2;
		}
	}
	if(len < 0)
	{
		perror("read error!\n");
		return 3;
	}
	close(src_fd);
	close(dest_fd);
	
	return 0;
}



