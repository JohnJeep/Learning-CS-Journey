CC = gcc
inc = -I ./include
obj = 02_multiprocess_server.o  wrap.o 
target = app
$(target) : $(obj)
	$(CC) $(obj) $(inc) -o $(target) -g

%.o : %.c
	$(CC) $(inc) -c $< -o $@ -g

.PHONY: clean
clean:
	rm -f $(target) $(obj)
