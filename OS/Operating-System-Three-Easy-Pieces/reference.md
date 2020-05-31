<!--
 * @Author: JohnJeep
 * @Date: 2020-05-18 23:22:46
 * @LastEditTime: 2020-05-30 18:12:08
 * @LastEditors: Please set LastEditors
 * @Description: Operation System Three Easy Pieces书中优秀的参考文献
--> 

## 线程API接口
- “Programming With Threads” by Steve Kleiman, Devang Shah, Bart Smaalders. Prentice Hall, January 1996.


## 线程锁
- “Cooperating sequential processes” by Edsger W. Dijkstra. 1968. Available online here:http://www.cs.utexas.edu/users/EWD/ewd01xx/EWD123.PDF. One of the early seminal papers. Discusses how Dijkstra posed the original concurrency problem, and Dekker’s solution.
- “glibc 2.9 (include Linux pthreads implementation)” by Many authors.. Available here:http://ftp.gnu.org/gnu/glibc. In particular, take a look at the nptl subdirectory where you will find most of the pthread support in Linux today.


## 条件变量
- “Information Streams Sharing a Finite Buffer” by E.W. Dijkstra. Information ProcessingLetters 1: 179180, 1972. Available: http://www.cs.utexas.edu/users/EWD/ewd03xx/EWD329.PDF. The famous paper that introduced the producer/consumer problem.
- “My recollections of operating system design” by E.W. Dijkstra. April, 2001. Available: http://www.cs.utexas.edu/users/EWD/ewd13xx/EWD1303.PDF. A fascinating read for those of you interested in how the pioneers of our field came up with some very basic and fundamental concepts, including ideas like “interrupts” and even “a stack”!


## 信号量
- “Hierarchical ordering of sequential processes” by E.W. Dijkstra. Available online here: http://www.cs.utexas.edu/users/EWD/ewd03xx/EWD310.PDF. Presents numerous concurrency problems, including Dining Philosophers. The wikipedia page about this problem is also useful.
- “The Little Book of Semaphores” by A.B. Downey. Available at the following site: http://greenteapress.com/semaphores/. A nice (and free!) book about semaphores. Lotsof fun problems to solve, if you like that sort of thing


## 并发问题
- “System Deadlocks” by E.G. Coffman, M.J. Elphick, A. Shoshani. ACM Computing Surveys, 3:2, June 1971. The classic paper outlining the conditions for deadlock and how you might go about dealing with it. There are certainly some earlier papers on this topic; see the references within this paper for details. 
- “Een algorithme ter voorkoming van de dodelijke omarming” by Edsger Dijkstra. 1964. Available: http://www.cs.utexas.edu/users/EWD/ewd01xx/EWD108.PDF. Indeed, not only did Dijkstra come up with a number of solutions to the deadlock problem, he was the first to note its existence, at least in written form. However, he called it the “deadly embrace”, which (thankfully) did not catch on.
- “Deadlock Immunity: Enabling Systems To Defend Against Deadlocks” by Horatiu Jula, Daniel Tralamazza, Cristian Zamfir, George Candea. OSDI ’08, San Diego, CA, December 2008. An excellent recent paper on deadlocks and how to avoid getting caught in the same ones over and over again in a particular system.
- “Linux File Memory Map Code” by Linus Torvalds and many others. Available online at: http://lxr.free-electrons.com/source/mm/filemap.c. Thanks to Michael Walfish (NYU) for pointing out this precious example. The real world, as you can see in this file, can be a bit more complex than the simple clarity found in textbooks...




