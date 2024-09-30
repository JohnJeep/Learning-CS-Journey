#! /bin/bash
# testing $@ and $*,for循环遍历输出变量

count=1
for t in "$*"
do 
	echo "\$* parameter #$count = $t"
	count=$[ $count + 1 ]
done

tmp=1
for s in "$@"
do
	echo "\$@ parameter #$tmp = $s"
	tmp=$[ $tmp + 1 ]
done

echo "parameter num: $#"
echo "shell name: $0"
