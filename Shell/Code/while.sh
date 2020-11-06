#! /bin/bash

str=hello
count=0

echo -n "Please input a word: "   # -n 打印不换行
count=$[$count+1]

read wd

while [ $count -lt 3 ] 
do
	if [ $wd != $str ]
	then
		read -p "Please input again: " wd   # -p: 输入时有提示信息
		count=$[ $count + 1 ]   # 没有++运算
		
		if [ $count -eq 3 ]
		then
			echo "Input counts exceed 3 times."
			return 0
		fi
	else
		echo "Input word is right."
		return 0    # 命令成功结束
	fi
done
