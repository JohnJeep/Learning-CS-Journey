CC = g++
inc = -I ./include
src = $(wildcard ./*.cpp)                        # 查找指定目录 ./ 下所有.cpp 的文件，并将函数的返回值赋值给 src 变量
# obj = main.o add.o sub.o multi.o
obj = $(patsubst ./%.cpp, ./%.o, $(src))         # 将src目录下所有的 .cpp 替换为 .o 文件
target = app_4
$(target): $(obj) 
	$(CC) $(obj) $(inc) -o $(target) 

%.o:%.cpp
	$(CC) $(inc) -c $< -o $@  


.PHONY: clean                                # 伪目标
clean:                                         # 本地没有生成一个clean的文件，只是执行了该命令
	rm -f $(target) $(obj)



