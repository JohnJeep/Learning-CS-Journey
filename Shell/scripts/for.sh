#! /bin/bash
# 利用ls命令判断当前目录下文件的的类型

for name in $(ls)
do
	printf "$name "
	if [ -d $name ]; then
		echo "It's a dir."
	elif [ -f $name ]; then
		echo "It's a file."
	else 
		echo "others."
	fi
done
