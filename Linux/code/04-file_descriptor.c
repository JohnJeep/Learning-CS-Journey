#include <unistd.h>
#include <string.h>

int main()
{
	char buf[20] = {0};

	read(0, buf, sizeof(buf));
	write(1, buf , sizeof(buf));

	return 0;
}


