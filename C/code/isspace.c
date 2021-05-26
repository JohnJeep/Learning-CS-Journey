/*
 * @Author: JohnJeep
 * @Date: 2020-01-06 09:14:19
 * @LastEditTime: 2021-05-26 22:17:50
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \C\isspace.c
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <ctype.h>

int main()
{
    char *p = "      isspace     ";
    int n = 0;
    int start = 0;
    int end = strlen(p) - 1;

    while (isspace(p[start]) && p[start] != '\0')
    {
        start++;
    }
    while (isspace(p[end]) && p[end] != '\0')
    {
        end--;
    }
    n = end - start + 1;
    printf("%d \n", strlen(p));
    printf("%d \n", n);

   return 0;
}


