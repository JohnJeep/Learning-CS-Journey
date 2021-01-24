/*
 * @Author: JohnJeep
 * @Date: 2021-01-22 21:44:07
 * @LastEditTime: 2021-01-24 09:50:33
 * @LastEditors: Please set LastEditors
 * @Description: 简单写一个Windows的Startup
 */
#include <windows.h>

int MyStartup() 
{
    int a = 10;
    HANDLE cHeap = HeapCreate(HEAP_NO_SERIALIZE, 0x010, 4000*1024);
    int* p = (int*)HeapAlloc(cHeap, HEAP_ZERO_MEMORY, 0x010);
    int i, j;

    for (i = 0; i < 100; i++)
    {
        for (j = 0; j < 100; j++, p++)
        {
            *p = i*100 + (j+1);
        }
        
    }
    
    MessageBoxA(NULL, p, "adcd", MB_OK);
    return 0;
}
