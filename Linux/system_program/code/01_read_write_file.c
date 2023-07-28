#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main(void)
{
	int fd; // file descriptor

	fd = open("helo.txt", O_RDWR|O_CREAT);   // file not exist,flags field set O_CREAT;O_RDWR: write and read
	if(fd == -1)
	{
		printf("open file failed!\n");
		return -1;
	}

	char string[] = "hello system programming";
	write(fd, string, sizeof(string)/sizeof(char));
	fsync(fd); // file descritor data write in the disk device

	char *buf = (char*)malloc(40);
	memset(buf, 0, 20); // buf init
	lseek(fd, 0, SEEK_SET); // reposition read/write file offset
	read(fd, buf, sizeof(string)/sizeof(char));
	printf("%s", buf);

	free(buf);  // free buf
	close(fd);
	return 0;



}

