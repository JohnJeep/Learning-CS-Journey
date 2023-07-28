/*
 * @Author: your name
 * @Date: 2020-06-06 20:00:24
 * @LastEditTime: 2020-06-06 20:02:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /system_program/segment_fault.c
 */ 
#include <stdio.h>

int main(int argc, char *argv[])
{	
	char *p = "abc";
	p[0] = 10;   // segment fault

	int a = 20;
	int var = 0;

//	var = a / 0;   // Floating point exception

	return 0;
}

