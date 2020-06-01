#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// declare structure
typedef struct
{
	char name[10];
	int age;
	float score;
}stu_type;


int main(int argc, char* argv[])
{
	stu_type stu;
	
	for(int i=0; i<2; i++)
	{
		printf("please input stu name, age, score\n");
		scanf("%s %d %f", stu.name, &stu.age, &stu.score);
	}

	FILE *fp;     // declare *fp
	if((fp = fopen("student.txt", "W+")) == NULL)
	{
		printf("fopen file failed!\n");
		return -1;
	}

	fwrite(&stu, sizeof(stu), 2, fp); // 向文件面写数据
	if(ferror(fp) != 0)
	{
		printf("fwrite file failed!\n");
		return -1;
	}

	fflush(fp);  // 将缓冲区的数据写入到硬盘
	rewind(fp);

	stu_type *buf = (stu_type*)malloc(2*sizeof(stu_type));
	fread(buf, sizeof(stu_type), 2, fp);

	if(ferror(fp) != 0)
	{
		printf("fread file failed!\n");
		clearerr(fp);
		return -1;
	}

	printf("姓名\t年龄\t分数\n");
	for(int i=0; i<2; i++)
	{
		printf("%s\t%d\t%f\n", buf[i].name, buf[i].age, buf[i].score);
	}
	fclose(fp);
	free(buf);
	buf = NULL;

	return 0;
}

