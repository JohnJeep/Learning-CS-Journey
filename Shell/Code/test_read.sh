#! /bin/bash
# testing if you can read a file

file=/etc/profile

if [ -f $file ]; then
	if [ -r $file ]; then
		tail $file
	else
		echo "Sorry, I am unable to read file."
	fi
else
	echo "Sorry, $file not exist."
fi

