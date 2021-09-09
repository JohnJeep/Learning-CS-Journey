/*
 * @Author: JohnJeep
 * @Date: 2020-08-17 13:59:25
 * @LastEditTime: 2021-09-09 23:27:49
 * @LastEditors: Windows10
 * @Description: 手写memcpy()函数，不用底层的库函数
 * https://www.cxyzjd.com/article/wyk71456/108424816
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <memory.h>

void *my_memcpy(void* dest, const void* src, size_t len)
{
    char *pdest;
    const char *psrc;

    // desc为空的话肯定不能拷贝到内存空间,src为空相当于没有拷贝，所以之间return
    if ((dest == NULL) || (src == NULL) || len <= 0) {
        return NULL;
    }
    
    if (dest > (src + len) || dest < src) {
        //没有内存重叠，从低地址开始复制
        pdest = (char*)dest;
        psrc = (char*)src; 
        while (len--) {
            *pdest++ = *psrc++;  // 先执行*，再执行++
        }
    }
    else {
        //有内存重叠，从高地址开始复制，减一是有 \0
        pdest = (char*)(dest + len - 1);
        psrc = (char*)(src + len - 1);

        while (len--) {
            *pdest-- = *psrc--;
        }
    }

    printf("%s\n", src);
    printf("%s\n", dest);
    
    return dest;    
}

// 优化版本

int main()
{
    const char* source = "hello";
    char *dest;
    
    printf("%d\n", sizeof(source));
    my_memcpy(dest, source, sizeof(source));

   return 0;
}