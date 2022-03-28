###################################################################
# 第一种方法：编写简单makefile文件
# 当文件被修改时，需要全部编译文件，包括没有修改的文件，很耗时间
# 顶级规则为源文件文件（.c）
# 
# app_1: main.c add.c sub.c multi.c
# 	gcc main.c add.c sub.c multi.c -o app_1 -I ./include
# 
###################################################################


###################################################################
# 第二种方法：编写简单makefile文件
# 只针对修改的文件有效，没有改变的文件编译一次后不会再被编译，节约了编译的时间
# 顶级规则为目标文件（.o）

# app_2: main.o add.o sub.o multi.o
# 	gcc main.o add.o sub.o multi.o -o app_2 -I ./include
# 
# main.o: main.c
# 	gcc main.c -I ./include -c
# 
# add.o: add.c
# 	gcc add.c -I ./include -c
# 
# sub.o: sub.c
# 	gcc sub.c -I ./include -c
# 
# multi.o: multi.c
# 	gcc multi.c -I ./include -c
###################################################################


###################################################################
# 第三种方法：编写简单makefile文件，进行优化
# 只针对修改的文件有效，没有改变的文件编译一次后不会再被编译，节约了编译的时间
# 顶级规则为目标文件（.o）
# 
# CC = gcc
# inc = -I ./include
# obj = main.o add.o sub.o multi.o
# target = app_3
# $(target): $(obj) 
# 	$(CC) $(obj) $(inc) -o $(target) 
# 
# %.o:%.c
# 	$(CC) $(inc) -c $< -o $@   
###################################################################



###################################################################
# 第四种方法：编写简单makefile文件
# 只针对修改的文件有效，没有改变的文件编译一次后不会再被编译，节约了编译的时间
# 顶级规则为目标文件（.o）

CC = gcc
inc = -I ./include
src = $(wildcard ./*.c)                        # 查找指定目录 ./ 下所有.c 的文件，并将函数的返回值赋值给 src 变量
# obj = main.o add.o sub.o multi.o
obj = $(patsubst ./%.c, ./%.o, $(src))         # 将src目录下所有的 .c 替换为 .o 文件
target = app_4
$(target): $(obj) 
	$(CC) $(obj) $(inc) -o $(target) 

%.o:%.c
	$(CC) $(inc) -c $< -o $@  


.PHONY: clean                                # 伪目标
clean:                                       # 本地没有生成一个clean的文件，只是执行了该命令
	-mkdir ./tmp/test.txt                    # - 表示该指令执行失败时，则忽略该指令 
	rm -f $(target) $(obj)
###################################################################
