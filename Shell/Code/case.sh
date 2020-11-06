#! /bin/bash

echo "Please input yes or no?"
read TMP     # 从内尺中读输入的变量
case "$TMP" in
	Yes|y|Y|YES)
		echo "It's ok.";;
	[nN]?)
		echo "It's not ok.";;
	*)
		echo "Input error.";;   # 默认执行    
esac
return 0;
echo "exec return."   # 返回后不执行

