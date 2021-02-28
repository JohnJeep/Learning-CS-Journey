/*
 * @Author: JohnJeep
 * @Date: 2021-02-27 17:59:48
 * @LastEditTime: 2021-02-27 22:33:32
 * @LastEditors: Please set LastEditors
 * @Description: 自己手写实现一个Strncpy函数
 *               标准库中函数的原型：char* strncpy(char* dest, const char* src, size_t len)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * 自己面试写的一个垃圾的strncpy的拷贝函数
 */
/*
void my_strncpy(char* dest, const char* src, size_t len)
{
    if ((dest == NULL) || (src == NULL) || (len < 0)) {
        return ;
    }
    if (dest > src + len) {
        while (*src != '\0' ) {
            *dest++ = *src++;
        }
    }
}
*/

// 正确的strncpy写法
char* my_strncpy(char* dest, const char* src, size_t len)
{
    if ((dest == NULL) || (src == NULL) || (len < 0)) {
        return NULL;
    }
    char* ptr = dest;

    while (*src != '\0' &&  len--) {
        *dest++ = *src++;
    }
    *dest = '\0';       // 结尾置为 \0

    return ptr;
}

int main(int argc, char *argv[])
{
    char destination[20] = {0};
    char source[] =  "hello";

    printf("str=%s\n", my_strncpy(destination, source, 3));

    return 0;
}