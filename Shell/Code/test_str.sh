#! /bin/bash

str1=love
# str2=you

echo -n "Please input str: "
read str2

if [ $str1 = $str2 ]; then
	echo "str1 equal str2."
else 
	echo "not equal."
fi
