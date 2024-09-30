#! /bin/bash

echo "Is it morning? Please answer yes or no."
read YES_OR_NO
if [ "$YES_OR_NO" = "yes" ]; then              # 取值必须使用""
	echo "Good moning!"
elif [ "$YES_OR_NO" = "no" ]; then
	echo "Good afternoon!" 
else
	echo "Sorry, $YES_OR_NO not recognized. Enter yes or no."
fi


