#! /bin/bash
# testing the read command

echo -n "Please input a word: "   # -n: 在本行输出，不换行打印
read str

echo "The word: $str"

if read -t 5 -p "Please input name during 5s: " name
then 
	echo "Input name: $name"
else
	echo "Time out."
fi
