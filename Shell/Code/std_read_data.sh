#! /bin/bash

# 读取Linux系统上文件里面保存的数据，每次调用read命令，它都会从文件中读取一行文本。
# 当文件中再没有内容时，read命令会退出并返回非零退出状态码。

line=1
cat test.sh | while read VALUE
do
	echo "line $line: $VALUE"
	line=$[ $line + 1]
done
echo "read finish."

